#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_sensor::color_detector_node — 相机颜色检测（每机一个）。

color_id.source=camera 的检测节点：订阅下视相机图像（~image_topic，缺省
camera/image_raw 相对机 namespace；真机 D435 覆写 /camera/color/image_raw），
跑 arena_sensor.color_detect 纯函数检测，发布与真值 tag_detector 完全同
契约的四话题（/detected/tag_id|type|color|confidence，哨兵 -1/255/255/0.0）。

诚实边界：本节点无 Scene、无 odom、无 TF——只吃图像+配置，真机同代码。
tag_id 恒 -1（零消费者；推断须读真值破边界）。type 由 scene_topology
color_map 反查（配置权威，非运行时真值）；同色多类型歧义发 255。

自门控：color_id.source=truth（回退显式）时启动即退、零 Publisher。
看门狗（2Hz）：图像龄 > stale_s 发哨兵——相机死后旧检测不得继续喂
bucket_select 计数（NO_DET 复位语义，bucket_select.py:31-33）。
"""
import numpy as np

import rospy
from std_msgs.msg import Int32, UInt8, Float32
from sensor_msgs.msg import Image

from zx2026_common import config as cfg
from arena_sensor.color_detect import detect, make_detect_params, NO_DET


class ColorDetectorNode:
    def __init__(self):
        rospy.init_node("color_detector_node", anonymous=True)
        self._active = False
        rules = cfg.load("competition_rules.yaml")
        source = (rules.get("color_id", {}) or {}).get("source", "camera")
        if source != "camera":
            rospy.loginfo("color_detector_node: disabled (color_id.source=%s)", source)
            return
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        ns = "/drone_%d" % self.drone_id
        image_topic = rospy.get_param("~image_topic", "camera/image_raw")
        if not image_topic.startswith("/"):
            image_topic = ns + "/" + image_topic

        cd = (rules.get("color_id", {}) or {}).get("camera_detector", {}) or {}
        self.stale_s = float(cd.get("stale_s", 0.4))
        self.params = make_detect_params(rules)

        # 颜色 → 类型反查（color_map 是配置权威源；歧义→255 交上层兜底）
        color_map = cfg.load("scene_topology.yaml").get("color_map", {}) or {}
        by_color = {}
        for t, c in color_map.items():
            by_color.setdefault(str(c), []).append(str(t))
        self.color_to_type = {}
        for c, types in by_color.items():
            self.color_to_type[cfg.color_to_uint8(c)] = (
                cfg.type_to_uint8(types[0]) if len(types) == 1 else NO_DET)

        self.pub_tag = rospy.Publisher(ns + "/detected/tag_id", Int32, queue_size=10)
        self.pub_type = rospy.Publisher(ns + "/detected/type", UInt8, queue_size=10)
        self.pub_color = rospy.Publisher(ns + "/detected/color", UInt8, queue_size=10)
        self.pub_conf = rospy.Publisher(ns + "/detected/confidence", Float32,
                                        queue_size=10)
        self.last_img_t = None
        rospy.Subscriber(image_topic, Image, self._on_image)
        rospy.Timer(rospy.Duration(0.5), self._watchdog)
        rospy.loginfo("color_detector_node: drone %d (%s) stale=%.2fs hsv s>=%d v>=%d",
                      self.drone_id, image_topic, self.stale_s,
                      self.params["s_min"], self.params["v_min"])
        self._active = True

    def _on_image(self, msg):
        if msg.encoding == "rgb8":          # 真机 D435 RGB
            img = np.ascontiguousarray(
                np.frombuffer(msg.data, np.uint8).reshape(
                    msg.height, msg.width, 3)[:, :, ::-1])
        else:                               # 仿真 bgr8
            if msg.encoding != "bgr8":
                rospy.logwarn_throttle(10.0,
                                       "drone %d unsupported encoding %s",
                                       self.drone_id, msg.encoding)
            img = np.frombuffer(msg.data, np.uint8).reshape(
                msg.height, msg.width, 3)
        color_u8, conf, _ = detect(img, self.params)
        type_u8 = self.color_to_type.get(color_u8, NO_DET) \
            if color_u8 != NO_DET else NO_DET
        self.pub_tag.publish(Int32(data=-1))
        self.pub_type.publish(UInt8(data=type_u8))
        self.pub_color.publish(UInt8(data=color_u8))
        self.pub_conf.publish(Float32(data=conf))
        self.last_img_t = rospy.get_time()

    def _watchdog(self, _evt):
        if self.last_img_t is None:
            return
        if rospy.get_time() - self.last_img_t > self.stale_s:
            self.pub_tag.publish(Int32(data=-1))
            self.pub_type.publish(UInt8(data=NO_DET))
            self.pub_color.publish(UInt8(data=NO_DET))
            self.pub_conf.publish(Float32(data=0.0))

    def run(self):
        if not self._active:
            return
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ColorDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

// arena_world_gazebo::drone_vel_plugin — Gazebo 11 无人机速度控制器插件
//
// 契约（与 arena_world 的 python 后端完全一致）：
//   订阅  /drone_<id>/vel_cmd  (geometry_msgs/Twist)  目标线速度 + yaw 速率
//   发布  /drone_<id>/odom      (nav_msgs/Odometry)   世界系位姿/速度
//   TF    world -> drone_<id>   （供 RViz/Foxglove 显示）
//
// 物理：对 base_link 施加合力 F = m * (kp*(v_cmd - v) + (0,0,g))，由 ODE 求解器
// 积分（含重力/碰撞）。yaw 由 SetAngularVel 直接给定。树/灌木/互撞均为真实碰撞。

#include <memory>
#include <string>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo
{

class DroneVelPlugin : public ModelPlugin
{
public:
  DroneVelPlugin() : _kp(3.0), _g(9.81), _yaw_rate(0.0), _last_pub(0.0) {}

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    _model = model;
    _link = model->GetLink("base_link");
    if (!_link)
    {
      auto links = model->GetLinks();
      if (!links.empty()) _link = links[0];
    }
    if (!_link)
    {
      gzerr << "DroneVelPlugin: no base_link in model [" << model->GetName()
            << "], plugin disabled\n";
      return;
    }
    _mass = _link->GetInertial()->Mass();

    if (sdf->HasElement("drone_id")) _id = sdf->Get<int>("drone_id");
    if (sdf->HasElement("kp")) _kp = sdf->Get<double>("kp");
    if (sdf->HasElement("g")) _g = sdf->Get<double>("g");

    _name = "drone_" + std::to_string(_id);
    _nh = std::make_shared<ros::NodeHandle>();
    _sub = _nh->subscribe("/" + _name + "/vel_cmd", 1,
                          &DroneVelPlugin::onVelCmd, this);
    _pub_odom = _nh->advertise<nav_msgs::Odometry>("/" + _name + "/odom", 10);
    _pub_path = _nh->advertise<nav_msgs::Path>("/" + _name + "/path", 2, true);
    _path.header.frame_id = "world";
    _path.poses.reserve(5000);
    _tf = std::make_shared<tf2_ros::TransformBroadcaster>();

    _update = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DroneVelPlugin::onUpdate, this));

    _ros_thread = std::make_shared<std::thread>([this]() {
      ros::Rate r(100);
      while (ros::ok() && _nh->ok())
      {
        _cbq.callAvailable(ros::WallDuration(0.01));
        r.sleep();
      }
    });

    gzmsg << "DroneVelPlugin[" << _name << "] mass=" << _mass
          << " kp=" << _kp << " loaded\n";
  }

private:
  void onVelCmd(const geometry_msgs::TwistConstPtr& msg)
  {
    _target.Set(msg->linear.x, msg->linear.y, msg->linear.z);
    _yaw_rate = msg->angular.z;
  }

  void onUpdate()
  {
    if (!_link || !_model) return;

    // ---- 速度跟踪力（世界系）：F = m * (kp*(v_cmd - v) + (0,0,g))
    ignition::math::Vector3d vel = _link->WorldLinearVel();
    ignition::math::Vector3d err = _target - vel;
    double em = err.Length();
    if (em > 8.0) err *= 8.0 / em;             // 限幅，防抖
    ignition::math::Vector3d acc = err * _kp;
    acc.Z() += _g;                              // 重力补偿
    _link->AddForce(acc * _mass);               // 世界系质心力

    // ---- yaw 速率
    _link->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, _yaw_rate));

    // ---- 位姿/odom 发布（20Hz）
    double t = _model->GetWorld()->SimTime().Double();
    if (t - _last_pub < 0.05) return;
    _last_pub = t;

    ignition::math::Pose3d pose = _model->WorldPose();
    ros::Time stamp(t);

    nav_msgs::Odometry od;
    od.header.stamp = stamp;
    od.header.frame_id = "world";
    od.child_frame_id = _name;
    od.pose.pose.position.x = pose.Pos().X();
    od.pose.pose.position.y = pose.Pos().Y();
    od.pose.pose.position.z = pose.Pos().Z();
    od.pose.pose.orientation.x = pose.Rot().X();
    od.pose.pose.orientation.y = pose.Rot().Y();
    od.pose.pose.orientation.z = pose.Rot().Z();
    od.pose.pose.orientation.w = pose.Rot().W();
    od.twist.twist.linear.x = vel.X();
    od.twist.twist.linear.y = vel.Y();
    od.twist.twist.linear.z = vel.Z();
    _pub_odom.publish(od);

    // 轨迹 Path（累积位姿，RViz 显示飞行路径）
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = "world";
    ps.pose = od.pose.pose;
    if (_path.poses.size() >= 5000) _path.poses.erase(_path.poses.begin());
    _path.poses.push_back(ps);
    _path.header.stamp = stamp;
    _pub_path.publish(_path);

    geometry_msgs::TransformStamped tr;
    tr.header.stamp = stamp;
    tr.header.frame_id = "world";
    tr.child_frame_id = _name;
    tr.transform.translation.x = pose.Pos().X();
    tr.transform.translation.y = pose.Pos().Y();
    tr.transform.translation.z = pose.Pos().Z();
    tr.transform.rotation.x = pose.Rot().X();
    tr.transform.rotation.y = pose.Rot().Y();
    tr.transform.rotation.z = pose.Rot().Z();
    tr.transform.rotation.w = pose.Rot().W();
    _tf->sendTransform(tr);
  }

  physics::ModelPtr _model;
  physics::LinkPtr _link;
  double _mass = 2.0;
  int _id = 0;
  double _kp, _g;
  std::string _name;

  ignition::math::Vector3d _target{0.0, 0.0, 0.0};
  double _yaw_rate = 0.0;
  double _last_pub = 0.0;

  std::shared_ptr<ros::NodeHandle> _nh;
  ros::CallbackQueue _cbq;
  ros::Subscriber _sub;
  ros::Publisher _pub_odom;
  ros::Publisher _pub_path;
  nav_msgs::Path _path;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf;
  std::shared_ptr<std::thread> _ros_thread;
  event::ConnectionPtr _update;
};

GZ_REGISTER_MODEL_PLUGIN(DroneVelPlugin)

}  // namespace gazebo

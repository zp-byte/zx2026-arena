# 单机任务描述：一架无人机携带一件物品送往投送区
uint8   drone_id          # 0..5
string  payload_type      # TYPE_A..TYPE_E（本机携带物品类型）
int32   drop_point_id     # 参考指派平台号（闭环 color_id 开启时仅审计用，
                          # 执行器不得据此导航——目标由机载识别择定）
string  marker_id         # 参考平台的显著标识（tag）
string  box_color         # 本机箱色（red/blue/yellow；空=旧消息，
                          # 接收端回退 color_map[payload_type]）
geometry_msgs/Pose takeoff_pose   # 起降点位姿（z 为 hover 高度）
float32 cruise_z          # 穿越区平飞高度
float32 max_vel           # 该机最大速度
int32   mission_seq       # 全局任务序号

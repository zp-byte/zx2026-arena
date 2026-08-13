# 单机任务描述：一架无人机携带一件物品送往一个投放点
uint8   drone_id          # 0..5
string  payload_type      # TYPE_A..TYPE_E（本机携带物品类型）
int32   drop_point_id     # 目标投放点编号
string  marker_id         # 目标投放点的显著标识（tag）
geometry_msgs/Pose takeoff_pose   # 起降点位姿（z 为 hover 高度）
float32 cruise_z          # 穿越区平飞高度
float32 max_vel           # 该机最大速度
int32   mission_seq       # 全局任务序号

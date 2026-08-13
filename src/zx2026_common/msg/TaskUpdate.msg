# 任务进度更新（节点间传递单机任务状态）
uint8   drone_id
string  payload_type
int32   drop_point_id
string  mission_state
uint8   drop_ok           # 0=未投放 1=正确 2=错误

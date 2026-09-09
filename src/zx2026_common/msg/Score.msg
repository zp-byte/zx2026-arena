# 单机计分明细
uint8   drone_id
int32   correct_drops      # 正确投放数
int32   wrong_drops        # 错误投放数
int32   score              # 累计得分
float64 completion_time    # 该机完赛时间（s，未完成=-1）
float64 first_correct_time # 首个正确投放时间（s，无=-1）
string  mission_state      # IDLE/TAKEOFF/EXECUTE/MATCH/DROP/RETURN/DONE/FAIL/RETIRED
int32   s1                 # 本机 S1 投放分（正确平台数×10，封顶逻辑在队级）
int8    retire_state       # 0=在赛 1=已退赛（出界/计划外落地/超高等条款）

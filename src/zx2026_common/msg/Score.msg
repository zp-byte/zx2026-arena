# 单机计分明细
uint8   drone_id
int32   correct_drops      # 正确投放数
int32   wrong_drops        # 错误投放数
int32   score              # 累计得分
float64 completion_time    # 该机完赛时间（s，未完成=-1）
float64 first_correct_time # 首个正确投放时间（s，无=-1）
string  mission_state      # IDLE/TAKEOFF/EXECUTE/MATCH/DROP/RETURN/DONE/FAIL

float64 x_err
float64 y_err

uint8 mark_type
# enum mark_type 地标类型枚举
uint8 mark_H = 0
uint8 mark_X = 1
uint8 mark_Num = 3

# if mark_type == 3, detected num
int64 num

bool success

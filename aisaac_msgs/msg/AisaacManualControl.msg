uint8 robot_id

### ドリブル操作
bool dribbler_on # ドリブラーのON/OFF

### キック操作
bool kick_straight # ストレートキック指令
bool kick_tip # tipキック指令

### ロボットの目標値
consai_msgs/State2D robot_velocity_target # ロボットの目標速度
bool emergency_stop # 緊急停止指令

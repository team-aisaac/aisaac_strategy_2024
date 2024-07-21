
uint8 robot_id
bool robot_position_init# ロボットの位置情報の初期化を行うか？

### ドリブル操作
bool dribble_state# ボールを検出したらドリブルするか？
bool dribble_advance# trueで前進ドリブル、falseで後退ドリブル
uint16 dribble_enabble_error# ボールをドリブルする目標値に対する許容誤差(mm)
geometry_msgs/Pose2D dribble_target_ball# ドリブルの目標地点. thetaは使わない


### キック操作
bool ball_kick_state# ボールを検出したらキックをするか？
uint16 ball_target_allowable_error# ボールをキックする目標値に対する許容誤差(mm)
bool free_kick_flag# フリーキックか？
bool ball_kick# ボールを実際にキックするか？
bool kick_straight# trueでストレートキック,falseでtipキック
geometry_msgs/Pose2D target_ball# ボールキックの目標地点. thetaは使わない


### ロボットの目標値
geometry_msgs/Pose2D robot_position_target #ロボットの目標地点 
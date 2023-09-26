
uint8 robot_id
bool dribble_state #ボールを検出したらドリブルするか？
bool dribble_advance #trueで前進ドリブル、falseで後退ドリブル
uint8 dribble_enabble_error  #ボールをドリブルする目標値に対する許容誤差(mm)

bool ball_kick_state #ボールを検出したらキックをするか？
uint8 ball_target_allowable_error #ボールをキックする目標値に対する許容誤差(mm)

bool free_kick_flag #フリーキックか？
bool ball_kick #ボールを実際にキックするか？
bool kick_straight #trueでストレートキック,falseでtipキック

consai_msgs/State2D ball_target #ボールキックの目標地点 
consai_msgs/State2D robot_target #ロボットの目標地点 

# aisaac_strategy_2024
戦略PC

## Info
### package
#### aisaac_consai_exapmles
- `/aisaac_referee`をpublishする(aisaac_msgs/msg/AisaacReferee.msg型)
- consai_ros2と比べて、referee_parser.pyしか意味のある変更はしていない
- PCスペックのせいか、None errorが出ることが多かったため、None処理を適宜加えてあるが、本質ではない
#### aisaac_consai_robot_controller
- `/aisaac_strategy`をpublishする(aisaac_msgs/msg/AisaacStrategy.msg型)
- consai_ros2と比べてcontroller_component.cppしか変更していない
#### aisaac_driver 
- mainboard_JO2023をROS2でシミュレーションする用。作成途中
- mainはまだ存在しない。callback.cをタイマーで回す予定
#### aisaac_msgs
- consai_ros2からaisaac用に出すmsg
- [仕様](https://docs.google.com/presentation/d/19eTkd5108suCrjKbtUJY7CcTNWBh3foa/)
- AisaacReferee : 試合状況。p8前半部分
- AisaacStrategy : 戦略的な指令。p8後半部分
- 仕様書との違い
    - robot_id (誰への司令か)
    - consai_msgs/State2D : x, y, thetaを持つ構造体



## Requirements

consai_ros2で試合のシミュレーションができる環境
https://github.com/SSL-Roots/consai_ros2.git

## Install
```
cd catkin_ws/src
git clone https://github.com/team-aisaac/aisaac_strategy_2024.git
```

## Build
```
colcon build --symlink-install
```

## Run
consai_ros2とほぼ同様
1. grSim起動
```
cd grSim/bin/
grSim
```
1. ssl-game-controller起動
```
cd ssl-game-controller
go run ../ssl-game-controller/cmd/ssl-game-controller/main.go
```
1. Auto referee起動
```
cd AutoReferee
bash run.sh
```
1. ros実行
```
ROS_DOMAIN_ID=1 ros2 launch aisaac_consai_robot_controller start.launch.py game:=true
ROS_DOMAIN_ID=2 ros2 launch aisaac_consai_robot_controller start.launch.py game:=true yellow:=true invert:=true
```



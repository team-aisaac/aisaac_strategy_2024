# aisaac_strategy_2024
戦略PC

## Info
### package
- aisaac_consai_robot_controller: consai_ros2/consai_robot_controllerにaisaac用のmsgを出力するコードを追記した. 作成途中
    - consai_ros2と比べてcontroller_component.cppしか変更していないが、ビルドの通し方がわからないため、他の不要ファイルも全部コピペした
- aisaac_driver : mainboard_JO2023をROS2でシミュレーションする用。作成途中
- aisaac_msgs


## Requirements

consai_ros2で試合のシミュレーションができる環境
https://github.com/SSL-Roots/consai_ros2.git


## Build
```
colcon build
```

## Run
[WARN] 現在作成途中なので動きません

consai_ros2とほぼ同様
1. grSim起動
1. ssl-game-controller起動
1. Auto referee起動
```
ROS_DOMAIN_ID=1 ros2 launch aisaac_consai_robot_controller start.launch.py game:=true
ROS_DOMAIN_ID=2 ros2 launch aisaac_consai_robot_controller start.launch.py game:=true yellow:=true invert:=true
```



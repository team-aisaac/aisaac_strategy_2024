// Copyright 2021 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <memory>
#include <vector>

#include "aisaac_consai_robot_controller/field_info_parser.hpp"
#include "aisaac_consai_robot_controller/geometry_tools.hpp"
#include "aisaac_consai_robot_controller/obstacle_ball.hpp"
#include "aisaac_consai_robot_controller/obstacle_environment.hpp"
#include "aisaac_consai_robot_controller/obstacle_robot.hpp"
#include "aisaac_consai_robot_controller/obstacle_typedef.hpp"
#include "aisaac_consai_robot_controller/prohibited_area.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"

namespace aisaac_consai_robot_controller
{

using ObstArea = obstacle::ProhibitedArea;
using ObstBall = obstacle::ObstacleBall;
using ObstEnv = obstacle::ObstacleEnvironment;
using ObstPos = obstacle::Position;
using ObstRadius = obstacle::Radius;
using ObstRobot = obstacle::ObstacleRobot;
using RobotId = robocup_ssl_msgs::msg::RobotId;
namespace tools = geometry_tools;
const double VISIBILITY_THRESHOLD = 0.01;
const double MAX_KICK_POWER_SHOOT = 5.5;  // m/s
const double MAX_KICK_POWER_PASS = 4.0;  // m/s
const double MIN_KICK_POWER_PASS = 2.0;  // m/s

FieldInfoParser::FieldInfoParser()
: invert_(false), team_is_yellow_(false)
{
  // 不正な値を参照しないように、フィールド情報の初期値をセットする
  geometry_ = std::make_shared<GeometryData>();
  geometry_->field.field_length = 12000;
  geometry_->field.field_width = 9000;
  geometry_->field.goal_width = 1800;
  geometry_->field.goal_depth = 180;
  geometry_->field.boundary_width = 300;
}

void FieldInfoParser::set_invert(const bool & invert)
{
  invert_ = invert;
}

void FieldInfoParser::set_team_is_yellow(const bool & team_is_yellow)
{
  team_is_yellow_ = team_is_yellow;
}

void FieldInfoParser::set_detection_tracked(const TrackedFrame::SharedPtr detection_tracked)
{
  detection_tracked_ = detection_tracked;
}

void FieldInfoParser::set_geometry(const GeometryData::SharedPtr geometry)
{
  geometry_ = geometry;
}

void FieldInfoParser::set_referee(const Referee::SharedPtr referee)
{
  referee_ = referee;
}

void FieldInfoParser::set_parsed_referee(const ParsedReferee::SharedPtr parsed_referee)
{
  parsed_referee_ = parsed_referee;
}

void FieldInfoParser::set_named_targets(const NamedTargets::SharedPtr msg)
{
  // トピックを受け取るたびに初期化する
  named_targets_.clear();

  for (std::size_t i = 0; i < msg->name.size(); ++i) {
    auto name = msg->name[i];
    auto pose = msg->pose[i];
    named_targets_[name] = pose;
  }
}

bool FieldInfoParser::extract_robot(
  const unsigned int robot_id, const bool team_is_yellow,
  TrackedRobot & my_robot) const
{
  // detection_trackedから指定された色とIDのロボット情報を抽出する
  // visibilityが低いときは情報が無いと判定する
  for (const auto & robot : detection_tracked_->robots) {
    if (robot_id != robot.robot_id.id) {
      continue;
    }
    bool is_yellow = team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_YELLOW;
    bool is_blue = !team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_BLUE;
    if (!is_yellow && !is_blue) {
      continue;
    }
    // if((team_is_yellow && robot.robot_id.team_color != RobotId::TEAM_COLOR_YELLOW) &&
    //    (!team_is_yellow && robot.robot_id.team_color != RobotId::TEAM_COLOR_BLUE)){
    //   continue;
    // }
    if (robot.visibility.size() == 0) {
      return false;
    }
    if (robot.visibility[0] < VISIBILITY_THRESHOLD) {
      return false;
    }

    my_robot = robot;
    break;
  }
  return true;
}

bool FieldInfoParser::extract_ball(TrackedBall & my_ball) const
{
  // detection_trackedからボール情報を抽出する
  // visibilityが低いときは情報が無いと判定する
  for (const auto & ball : detection_tracked_->balls) {
    if (ball.visibility.size() == 0) {
      return false;
    }
    if (ball.visibility[0] < VISIBILITY_THRESHOLD) {
      return false;
    }

    my_ball = ball;
    break;
  }
  return true;
}

bool FieldInfoParser::parse_goal(
  const std::shared_ptr<const RobotControl::Goal> goal,
  State & parsed_pose) const
{
  // RobotControlのgoalを解析し、目標姿勢を出力する
  // 解析に失敗したらfalseを返す
  // ここではキック処理や、レシーブ処理をしない

  // 目標姿勢を算出
  State target_pose;
  bool parse_succeeded = false;
  if (goal->pose.size() > 0) {
    if (parse_constraint_pose(goal->pose[0], target_pose)) {
      parse_succeeded = true;
    }
  }
  if (goal->line.size() > 0) {
    if (parse_constraint_line(goal->line[0], target_pose)) {
      parse_succeeded = true;
    }
  }

  if (parse_succeeded == false) {
    return false;
  }
  parsed_pose = target_pose;
  return true;
}

bool FieldInfoParser::parse_goal(
  const std::shared_ptr<const RobotControl::Goal> goal,
  const TrackedRobot & my_robot, State & parsed_pose, State & final_goal_pose,
  double & kick_power, double & dribble_power) const
{
  // RobotControlのgoalを解析し、目標姿勢を出力する
  // 解析に失敗したらfalseを返す
  // 衝突回避やキック、レシーブの処理も実施する

  // 目標姿勢を算出
  if (!parse_goal(goal, parsed_pose)) {
    return false;
  }

  // ボール受取や、衝突回避に影響されない、最終目標姿勢を格納する
  final_goal_pose = parsed_pose;

  // 以下、ボールが関わる処理のためボール情報を取得する
  TrackedBall ball;
  if (!extract_ball(ball)) {
    // ボール情報を取得できなくても正常終了
    return true;
  }

  State target;
  bool result = false;
  if (goal->reflect_shoot && parse_constraint_xy(goal->kick_target, target.x, target.y) ) {
    // ボールを受け取りながら目標へ向かって蹴るリフレクトシュート
    result = reflect_kick(
      target, my_robot, ball, goal->kick_pass, parsed_pose, kick_power,
      dribble_power);
  }

  if (goal->receive_ball && result == false) {
    // 転がっているボールを受け取る
    result = receive_ball(my_robot, ball, parsed_pose, dribble_power);
  }

  if (tools::distance(tools::pose_state(ball), parsed_pose) < 0.7 && result == false) {
    // 目標姿勢とボールが近ければ、ボールを操作する
    if (goal->kick_enable &&
      parse_constraint_xy(goal->kick_target, target.x, target.y))
    {
      parse_kick(
        target, my_robot, ball, goal->kick_pass, goal->kick_setplay, parsed_pose,
        kick_power, dribble_power);
    } else if (goal->dribble_enable &&  // NOLINT
      parse_constraint_xy(goal->dribble_target, target.x, target.y))
    {
      parse_dribble(target, my_robot, ball, parsed_pose, dribble_power);
    }
  }

  return true;
}

std::vector<unsigned int> FieldInfoParser::active_robot_id_list(const bool team_is_yellow) const
{
  // 存在しているロボットのIDリストを返す
  std::vector<unsigned int> id_list;
  if (detection_tracked_) {
    for (const auto & robot : detection_tracked_->robots) {
      bool is_yellow = team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_YELLOW;
      bool is_blue = !team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_BLUE;
      if (!is_yellow && !is_blue) {
        continue;
      }

      if (robot.visibility.size() == 0) {
        continue;
      }
      if (robot.visibility[0] < VISIBILITY_THRESHOLD) {
        continue;
      }

      id_list.push_back(robot.robot_id.id);
    }
  }
  return id_list;
}

State FieldInfoParser::modify_goal_pose_to_avoid_obstacles(
  const std::shared_ptr<const RobotControl::Goal> goal,
  const TrackedRobot & my_robot,
  const State & goal_pose, const State & final_goal_pose) const
{
  State avoidance_pose = goal_pose;

  if (!goal->avoid_obstacles) {
    return avoidance_pose;
  }

  TrackedBall ball;
  if (!extract_ball(ball)) {
    return avoidance_pose;
  }

  bool avoid_ball = false;
  // STOP_GAME中はボールから離れる
  if (parsed_referee_->is_our_setplay == false && parsed_referee_->is_inplay == false) {
    avoid_ball = true;
  }
  avoid_obstacles(my_robot, goal_pose, ball, avoid_ball, avoidance_pose);

  // ボールプレイスメントエリアを回避する
  if (referee_->command == Referee::COMMAND_BALL_PLACEMENT_YELLOW ||
    referee_->command == Referee::COMMAND_BALL_PLACEMENT_BLUE)
  {
    if (referee_->designated_position.size() > 0) {
      State designated_position;
      designated_position.x = referee_->designated_position[0].x * 0.001;  // mm to meters
      designated_position.y = referee_->designated_position[0].y * 0.001;  // mm to meters

      // サイド反転
      if (invert_) {
        designated_position.x *= -1.0;
        designated_position.y *= -1.0;
      }

      bool is_our_placement =
        (referee_->command == Referee::COMMAND_BALL_PLACEMENT_YELLOW &&
        team_is_yellow_ == true) ||
        (referee_->command == Referee::COMMAND_BALL_PLACEMENT_BLUE && team_is_yellow_ == false);
      bool avoid_kick_receive_area = true;
      // 自チームのプレースメント時は、キック、レシーブエリアを避けない
      if (is_our_placement) {
        avoid_kick_receive_area = false;
      }
      if (goal->avoid_placement) {
        avoid_placement_area(
          my_robot, avoidance_pose, ball, avoid_kick_receive_area,
          designated_position, avoidance_pose);
      }
    }
  }

  // STOP中、プレースメント中は目標位置とロボットの重なりを回避する
  if (referee_->command == Referee::COMMAND_BALL_PLACEMENT_YELLOW ||
    referee_->command == Referee::COMMAND_BALL_PLACEMENT_BLUE ||
    referee_->command == Referee::COMMAND_STOP)
  {
    avoid_robots(my_robot, avoidance_pose, avoidance_pose);
  }

  // STOP_GAME中はボールから離れる
  if (avoid_ball) {
    avoid_ball_500mm(final_goal_pose, avoidance_pose, ball, avoidance_pose);
  }

  return avoidance_pose;
}

obstacle::ObstacleEnvironment FieldInfoParser::get_obstacle_environment(
  const std::shared_ptr<const RobotControl::Goal> goal,
  const TrackedRobot & my_robot) const
{
  constexpr ObstRadius BALL_RADIUS(0.0215);
  constexpr ObstRadius ROBOT_RADIUS(0.09);

  // goalから障害物環境を作成する
  ObstEnv environment;

  // 衝突回避しない場合は空の環境を返す
  if (!goal->avoid_obstacles) {
    return environment;
  }

  // ロボット、ボール情報がない場合は空の環境を返す
  if (!detection_tracked_) {
    return environment;
  }

  // インプレイと自チームセットプレイ以外ではボールから離れる
  // TODO(ShotaAk): ここは戦略側で判断できそう
  if (parsed_referee_) {
    if (parsed_referee_->is_our_setplay == false && parsed_referee_->is_inplay == false) {
      TrackedBall ball;
      if (extract_ball(ball)) {
        environment.append_obstacle_ball(ObstBall(ObstPos(ball.pos.x, ball.pos.y), BALL_RADIUS));
      }
    }
  }

  // ロボットを障害物として扱う
  for (const auto & robot : detection_tracked_->robots) {
    if (robot.visibility.size() == 0) {
      continue;
    }
    if (robot.visibility[0] < VISIBILITY_THRESHOLD) {
      continue;
    }

    // 自身の情報は除外する
    if (robot.robot_id.id == my_robot.robot_id.id &&
      robot.robot_id.team_color == my_robot.robot_id.team_color)
    {
      continue;
    }

    environment.append_obstacle_robot(
      ObstRobot(ObstPos(robot.pos.x, robot.pos.y), ROBOT_RADIUS));
  }

  // TODO(ShotaAk): ボールプレースメント回避エリアとディフェンスエリアを障害物に追加する

  return environment;
}

bool FieldInfoParser::parse_constraint_pose(const ConstraintPose & pose, State & parsed_pose) const
{
  double parsed_x, parsed_y;
  if (!parse_constraint_xy(pose.xy, parsed_x, parsed_y)) {
    return false;
  }
  parsed_x += pose.offset.x;
  parsed_y += pose.offset.y;

  double parsed_theta;
  if (!parse_constraint_theta(pose.theta, parsed_x, parsed_y, parsed_theta)) {
    return false;
  }

  parsed_theta = tools::normalize_theta(parsed_theta + pose.offset.theta);

  parsed_pose.x = parsed_x;
  parsed_pose.y = parsed_y;
  parsed_pose.theta = parsed_theta;

  return true;
}

bool FieldInfoParser::parse_constraint_line(
  const ConstraintLine & line, State & parsed_pose) const
{
  State p1, p2;
  if (!parse_constraint_xy(line.p1, p1.x, p1.y)) {
    return false;
  }
  if (!parse_constraint_xy(line.p2, p2.x, p2.y)) {
    return false;
  }

  State p3, p4;
  bool has_p3_p4 = false;
  if (line.p3.size() > 0 && line.p4.size() > 0) {
    if (parse_constraint_xy(line.p3[0], p3.x, p3.y) &&
      parse_constraint_xy(line.p4[0], p4.x, p4.y))
    {
      has_p3_p4 = true;
    }
  }

  // 直線p1->p2の座標系を作成
  auto angle_p1_to_p2 = tools::calc_angle(p1, p2);
  tools::Trans trans_1to2(p1, angle_p1_to_p2);
  if (has_p3_p4) {
    // p1 ~ p4がセットされていれば、
    // 直線p1->p2上で、直線p3->p4と交わるところを目標位置とする
    State intersection = tools::intersection(p1, p2, p3, p4);
    // 交点が取れなければp1を目標位置とする
    if (!std::isfinite(intersection.x) || !std::isfinite(intersection.y)) {
      parsed_pose = p1;
    } else {
      auto parsed_pose_1to2 = trans_1to2.transform(intersection);
      auto p2_1to2 = trans_1to2.transform(p2);
      // 交点が直線p1->p2をはみ出る場合は、p1 or p2に置き換える
      if (parsed_pose_1to2.x < 0.0) {
        parsed_pose_1to2.x = 0.0;
      } else if (parsed_pose_1to2.x > p2_1to2.x) {
        parsed_pose_1to2.x = p2_1to2.x;
      }
      // オフセットを加算する。オフセットによりp1, p2をはみ出ることが可能
      if (line.offset_intersection_to_p2.size() > 0) {
        parsed_pose_1to2.x += line.offset_intersection_to_p2[0];
      }
      // 座標系をもとに戻す
      parsed_pose = trans_1to2.inverted_transform(parsed_pose_1to2);
    }
  } else {
    // 直線p1->p2上で、p1からdistanceだけ離れた位置を目標位置する
    parsed_pose = trans_1to2.inverted_transform(line.distance, 0, 0);
  }

  if (!parse_constraint_theta(
      line.theta, parsed_pose.x, parsed_pose.y,
      parsed_pose.theta))
  {
    return false;
  }


  return true;
}

bool FieldInfoParser::parse_constraint_xy(
  const ConstraintXY & xy, double & parsed_x,
  double & parsed_y) const
{
  State object_pose;
  if (xy.object.size() > 0) {
    if (parse_constraint_object(xy.object[0], object_pose)) {
      parsed_x = object_pose.x;
      parsed_y = object_pose.y;
    } else {
      return false;
    }
  }

  if (xy.value_x.size() > 0) {
    parsed_x = xy.value_x[0];
  }

  if (xy.value_y.size() > 0) {
    parsed_y = xy.value_y[0];
  }

  // フィールドサイズに対してx, yが-1 ~ 1に正規化されている
  if (xy.normalized) {
    // フィールド情報がなければfalse
    if (!geometry_) {
      return false;
    }

    parsed_x *= geometry_->field.field_length * 0.5 * 0.001;
    parsed_y *= geometry_->field.field_width * 0.5 * 0.001;
  }
  return true;
}

bool FieldInfoParser::parse_constraint_theta(
  const ConstraintTheta & theta, const double goal_x,
  const double goal_y, double & parsed_theta) const
{
  State object_pose;
  if (theta.object.size() > 0) {
    if (parse_constraint_object(theta.object[0], object_pose)) {
      if (theta.param == ConstraintTheta::PARAM_THETA) {
        parsed_theta = object_pose.theta;
        return true;
      } else if (theta.param == ConstraintTheta::PARAM_LOOK_TO) {
        State goal_pose;
        goal_pose.x = goal_x;
        goal_pose.y = goal_y;
        parsed_theta = tools::calc_angle(goal_pose, object_pose);
        return true;
      } else if (theta.param == ConstraintTheta::PARAM_LOOK_FROM) {
        State goal_pose;
        goal_pose.x = goal_x;
        goal_pose.y = goal_y;
        parsed_theta = tools::calc_angle(object_pose, goal_pose);
        return true;
      }
    }
  }

  if (theta.value_theta.size() > 0) {
    parsed_theta = theta.value_theta[0];
    return true;
  }

  return false;
}

bool FieldInfoParser::parse_constraint_object(
  const ConstraintObject & object,
  State & object_pose) const
{
  TrackedBall ball;
  TrackedRobot robot;

  // NOLINTについて
  // ament_uncrustifyとament_cpplintが競合するので、lintのチェックをスキップする
  // Ref: https://github.com/ament/ament_lint/issues/158
  if (object.type == ConstraintObject::BALL && extract_ball(ball)) {
    object_pose.x = ball.pos.x;
    object_pose.y = ball.pos.y;
    return true;
  } else if (  // NOLINT
    (object.type == ConstraintObject::BLUE_ROBOT && extract_robot(object.robot_id, false, robot)) ||
    (object.type == ConstraintObject::YELLOW_ROBOT &&
    extract_robot(object.robot_id, true, robot)) ||
    (object.type == ConstraintObject::OUR_ROBOT &&
    extract_robot(object.robot_id, team_is_yellow_, robot)) ||
    (object.type == ConstraintObject::THEIR_ROBOT &&
    extract_robot(object.robot_id, !team_is_yellow_, robot)))
  {
    object_pose.x = robot.pos.x;
    object_pose.y = robot.pos.y;
    object_pose.theta = robot.orientation;
    return true;
  } else if (object.type == ConstraintObject::NAMED_TARGET &&  // NOLINT
    named_targets_.find(object.name) != named_targets_.end())
  {
    object_pose = named_targets_.at(object.name);
    return true;
  } else if (object.type == ConstraintObject::OUR_GOAL) {
    object_pose.x = -geometry_->field.field_length * 0.5 * 0.001;
    object_pose.y = 0.0;
    return true;
  } else if (object.type == ConstraintObject::THEIR_GOAL) {
    object_pose.x = geometry_->field.field_length * 0.5 * 0.001;
    object_pose.y = 0.0;
    return true;
  }

  return false;
}

bool FieldInfoParser::parse_kick(
  const State & kick_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const bool & kick_pass, const bool & kick_setplay,
  State & parsed_pose, double & parsed_kick_power, double & parsed_dribble_power) const
{
  const double DRIBBLE_DISTANCE = 0.0;
  const double DRIBBLE_POWER = 1.0;
  bool need_kick = false;
  bool need_dribble = false;

  auto robot_pose = tools::pose_state(my_robot);
  double distance = tools::distance(robot_pose, kick_target);

  // パス速度を設定
  double kick_power_pass = MAX_KICK_POWER_PASS;
  if (distance < 1.0) {
    kick_power_pass = MIN_KICK_POWER_PASS;
  } else {
    // 2.0m離れてたら 2.0 m/sでける
    kick_power_pass = std::min(distance, MAX_KICK_POWER_PASS);
  }

  if (kick_setplay) {
    control_ball_at_setplay(kick_target, my_robot, ball, parsed_pose, need_kick, need_dribble);
  } else {
    control_ball(
      kick_target, my_robot, ball, DRIBBLE_DISTANCE, parsed_pose, need_kick,
      need_dribble);
  }

  if (need_dribble) {
    parsed_dribble_power = DRIBBLE_POWER;
  } else {
    parsed_dribble_power = 0.0;
  }

  if (need_kick) {
    if (kick_pass) {
      parsed_kick_power = kick_power_pass;
    } else {
      parsed_kick_power = MAX_KICK_POWER_SHOOT;
    }
  } else {
    parsed_kick_power = 0.0;
  }
  return true;
}

bool FieldInfoParser::parse_dribble(
  const State & dribble_target, const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_dribble_power) const
{
  const double DRIBBLE_DISTANCE = 0.15;
  const double DRIBBLE_POWER = 1.0;
  bool need_kick = false;
  bool need_dribble = false;

  control_ball(
    dribble_target, my_robot, ball, DRIBBLE_DISTANCE, parsed_pose, need_kick,
    need_dribble);

  if (need_dribble) {
    parsed_dribble_power = DRIBBLE_POWER;
  } else {
    parsed_dribble_power = 0.0;
  }
  return true;
}

bool FieldInfoParser::control_ball(
  const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const double & dribble_distance, State & parsed_pose, bool & need_kick, bool & need_dribble) const
{
  // ボールを操作する関数
  // キック、パス、ドリブルの操作が可能

  // 変数の初期化
  need_kick = false;
  need_dribble = false;

  auto ball_pose = tools::pose_state(ball);
  auto robot_pose = tools::pose_state(my_robot);

  // Ref: https://ssl.robocup.org/wp-content/uploads/2019/01/2014_ETDP_RoboDragons.pdf
  // // ボールからターゲットを見た座標系を生成
  const double BALL_RADIUS = 0.043 * 0.5;
  const double ROBOT_RADIUS = 0.180 * 0.5;
  const double MAX_X = BALL_RADIUS + 0.3;
  const double MAX_Y = BALL_RADIUS + ROBOT_RADIUS + 0.3;
  const double PIVOT_Y = 0.1;  // meters
  const double PHI = 60.0;  // degerees
  const double THETA_CORRECTION_THRESHOLD = 10.0;  // degrees

  auto angle_ball_to_target = tools::calc_angle(ball_pose, target);
  tools::Trans trans_BtoT(ball_pose, angle_ball_to_target);
  auto robot_pose_BtoT = trans_BtoT.transform(robot_pose);

  // ボールの斜め後ろへ近づく
  // 最終的にはBtoT上の(MAX_X, MAX_Y)に到達する
  if (robot_pose_BtoT.x > 0.0) {
    parsed_pose = trans_BtoT.inverted_transform(
      -MAX_X, std::copysign(MAX_Y, robot_pose_BtoT.y), 0.0);
    return true;
  }

  // ボールの裏へ回るためのピボットを生成
  State pivot_pose_BtoT;
  pivot_pose_BtoT.x = 0.0;
  pivot_pose_BtoT.y = PIVOT_Y;
  // ロボットの位置に合わせてpivotを反転
  if (robot_pose_BtoT.y < 0) {
    pivot_pose_BtoT.y *= -1.0;
  }

  // pivotを軸にした角度thetaを計算
  const double angle_pivot_to_robot_BtoT = tools::calc_angle(
    pivot_pose_BtoT, robot_pose_BtoT);
  double theta = std::fabs(angle_pivot_to_robot_BtoT) - M_PI_2;
  // pivotより内側に回り込んだ場合、thetaは90度を超える
  if (std::fabs(robot_pose_BtoT.y) < PIVOT_Y) {
    theta = std::fabs(angle_pivot_to_robot_BtoT + std::copysign(M_PI + M_PI_2, robot_pose_BtoT.y));
  }

  // ドリブラーを動かしながらボールの裏へ回る
  // 最終的にはBtoT上の(MAX_X, 0)に到達する
  // ロボットがボールを見るように姿勢も修正する
  need_dribble = true;

  tools::Trans trans_RtoB(robot_pose, tools::calc_angle(robot_pose, ball_pose));
  const auto robot_theta_RtoB = trans_RtoB.transform_theta(robot_pose.theta);
  const bool need_theta_correction =
    std::fabs(robot_theta_RtoB) > tools::to_radians(THETA_CORRECTION_THRESHOLD);
  const bool need_step2_motion = tools::to_degrees(theta) < PHI;
  if (need_step2_motion || need_theta_correction) {
    const double gain = 1.0 - std::clamp(tools::to_degrees(theta) / PHI, 0.0, 1.0);
    parsed_pose = trans_BtoT.inverted_transform(
      -MAX_X, std::copysign(MAX_Y, robot_pose_BtoT.y) * gain, 0.0);
    parsed_pose.theta = trans_RtoB.inverted_transform_theta(0.0);
    return true;
  }

  // ボールへ向かう
  // 最終的にはBtoT上で(-ボールの直径, 0)に到達する
  double gain = std::clamp(theta / (M_PI - tools::to_radians(PHI)), 0.0, 1.0);
  // 移動速度を早くするため、gainの分解能を荒くする
  gain = std::round(gain * 10) / 10;
  double distance_x = (-2.0 * BALL_RADIUS + MAX_X) * gain - MAX_X;

  // ボールの裏に回りきったら前進する
  if (gain >= 0.8) {
    distance_x = dribble_distance;
    need_kick = true;
  }
  parsed_pose = trans_BtoT.inverted_transform(distance_x, 0.0, 0.0);
  return true;
}

bool FieldInfoParser::control_ball_at_setplay(
  const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, bool & need_kick, bool & need_dribble) const
{
  // セットプレイ用の落ち着いたキックを実施

  // 変数の初期化
  need_kick = false;
  need_dribble = false;

  auto ball_pose = tools::pose_state(ball);
  auto robot_pose = tools::pose_state(my_robot);

  // ボールからターゲットを見た座標系を生成
  auto angle_ball_to_target = tools::calc_angle(ball_pose, target);
  tools::Trans trans_BtoT(ball_pose, angle_ball_to_target);
  auto robot_pose_BtoT = trans_BtoT.transform(robot_pose);

  // ロボットがボールの裏側に回ったらcan_kick
  bool can_kick = robot_pose_BtoT.x < 0.01 && std::fabs(robot_pose_BtoT.y) < 0.05;

  if (can_kick) {
    parsed_pose = trans_BtoT.inverted_transform(0.02, 0.0, 0.0);
    need_kick = true;
  } else {
    parsed_pose = trans_BtoT.inverted_transform(-0.3, 0.0, 0.0);
  }
  return true;
}


bool FieldInfoParser::receive_ball(
  const TrackedRobot & my_robot, const TrackedBall & ball,
  State & parsed_pose, double & parsed_dribble_power) const
{
  // 転がっているボールを受け取る
  const double DRIBBLE_POWER = 1.0;

  // ボール情報に速度情報がなければ終了
  if (ball.vel.size() == 0) {
    return false;
  }

  State velocity;
  velocity.x = ball.vel[0].x;
  velocity.y = ball.vel[0].y;
  // ボール速度が一定値以下であれば終了
  if (std::hypot(velocity.x, velocity.y) <= 0.5) {
    return false;
  }

  auto ball_pose = tools::pose_state(ball);
  auto robot_pose = tools::pose_state(my_robot);
  auto angle_velocity = std::atan2(velocity.y, velocity.x);
  tools::Trans trans_BtoV(ball_pose, angle_velocity);

  auto robot_pose_BtoV = trans_BtoV.transform(robot_pose);

  // ボールの軌道から離れていたら終了
  if (std::fabs(robot_pose_BtoV.y) > 1.0 || robot_pose_BtoV.x < 0.0) {
    return false;
  }

  // ボールの軌道上に移動する
  robot_pose_BtoV.y = 0.0;
  robot_pose_BtoV.theta = M_PI;
  parsed_pose = trans_BtoV.inverted_transform(robot_pose_BtoV);
  parsed_dribble_power = DRIBBLE_POWER;

  return true;
}

bool FieldInfoParser::reflect_kick(
  const State & target, const TrackedRobot & my_robot, const TrackedBall & ball,
  const bool & kick_pass, State & parsed_pose, double & parsed_kick_power,
  double & parsed_dribble_power) const
{
  // 転がっているボールの軌道上に移動し、targetに向かって蹴る
  // targetを狙えない場合は、蹴らずにボールを受け取る

  const double MIN_VELOCITY_THRESHOLD = 0.5;  // m/s ボールの最低動作速度
  const double MAX_DISTANCE_TO_RECEIVE = 1.0;  // meters ボールを受け取る最長距離
  const double DISTANCE_TO_DRIBBLER = 0.055;  // meters ロボットの中心からドリブラーまでの距離
  const double CAN_REFLECT_ANGLE = 60.0;  // degress リフレクトできる最大角度

  // パラメータを初期化
  parsed_dribble_power = 0.0;
  parsed_kick_power = 0.0;

  // ボール情報に速度情報がなければ終了
  if (ball.vel.size() == 0) {
    return false;
  }

  State velocity;
  velocity.x = ball.vel[0].x;
  velocity.y = ball.vel[0].y;
  auto velocity_norm = std::hypot(velocity.x, velocity.y);
  // ボール速度が一定値以下であれば終了
  if (velocity_norm <= MIN_VELOCITY_THRESHOLD) {
    return false;
  }

  // ロボット座標と、ロボットのドリブラー座標を作成
  auto robot_pose = tools::pose_state(my_robot);
  auto dribbler_pose = robot_pose;
  dribbler_pose.x += DISTANCE_TO_DRIBBLER * std::cos(robot_pose.theta);
  dribbler_pose.y += DISTANCE_TO_DRIBBLER * std::sin(robot_pose.theta);

  // ボールを中心にボール速度方向への座標系を作成
  auto ball_pose = tools::pose_state(ball);
  auto angle_velocity = std::atan2(velocity.y, velocity.x);
  tools::Trans trans_BtoV(ball_pose, angle_velocity);

  auto dribbler_pose_BtoV = trans_BtoV.transform(dribbler_pose);

  // ロボットがボールの軌道から離れていたら終了
  if (std::fabs(dribbler_pose_BtoV.y) > MAX_DISTANCE_TO_RECEIVE || dribbler_pose_BtoV.x < 0.0) {
    return false;
  }

  // ドリブラーをボール軌道上へ移動する
  dribbler_pose_BtoV.y = 0.0;

  // targetへの角度を計算し、リフレクトシュートできるか判定する
  auto target_BtoV = trans_BtoV.transform(target);
  auto angle_dribbler_to_target_BtoV = tools::calc_angle(dribbler_pose_BtoV, target_BtoV);
  if (std::fabs(angle_dribbler_to_target_BtoV) < tools::to_radians(180 - CAN_REFLECT_ANGLE)) {
    // リフレクトシュートできないため終了
    return false;
  }

  auto receiving_dribbler_pose = trans_BtoV.inverted_transform(dribbler_pose_BtoV);
  auto angle_dribbler_to_target = tools::calc_angle(receiving_dribbler_pose, target);
  tools::Trans trans_DtoT(receiving_dribbler_pose, angle_dribbler_to_target);
  auto ball_pose_DtoT = trans_DtoT.transform(ball_pose);
  auto angle_dribbler_to_ball_DtoT = std::atan2(ball_pose_DtoT.y, ball_pose_DtoT.x);

  // リフレクトシュート目標位置を生成
  // TODO(Roots) :ボール速度、キック速度のベクトルを結合して、目標角度を求める
  auto target_angle_DtoT = angle_dribbler_to_ball_DtoT * 0.7 * velocity_norm / 6.5;
  parsed_pose = trans_DtoT.inverted_transform(-DISTANCE_TO_DRIBBLER, 0.0, target_angle_DtoT);
  // キックパワーをセット
  if (kick_pass) {
    parsed_kick_power = MAX_KICK_POWER_PASS;
  } else {
    parsed_kick_power = MAX_KICK_POWER_SHOOT;
  }

  return true;
}

bool FieldInfoParser::avoid_obstacles(
  const TrackedRobot & my_robot, const State & goal_pose, const TrackedBall & ball,
  const bool & avoid_ball, State & avoidance_pose) const
{
  // 障害物を回避するposeを生成する
  // 全ロボット情報を検索し、
  // 自己位置(my_robot)と目標位置(goal_pose)間に存在する、
  // 自己位置に最も近い障害物付近に回避点を生成し、その回避点を新しい目標位置とする

  const double VISIBILITY_THRESHOLD = 0.01;  // 0.0 ~ 1.0
  // 自身から直進方向に何m離れたロボットを障害物と判定するか
  const double OBSTACLE_DETECTION_X = 0.5;
  // 自身から直進方向に対して左右何m離れたロボットを障害物と判定するか
  const double OBSTACLE_DETECTION_Y_ROBOT = 0.3;
  const double OBSTACLE_DETECTION_Y_BALL = 0.2;
  // 回避の相対位置
  const double AVOIDANCE_POS_X_SHORT = 0.1;
  const double AVOIDANCE_POS_X_LONG = 0.2;
  const double AVOIDANCE_POS_Y = 0.4;

  // 相対的な回避位置
  double avoidance_pos_x = 0.0;
  double avoidance_pos_y = 0.0;

  // 障害物の検索ループの
  const int NUM_ITERATIONS = 6;

  // 自己位置の格納
  auto my_robot_pose = tools::pose_state(my_robot);
  // 回避位置の初期値を目標位置にする
  avoidance_pose = goal_pose;

  std::shared_ptr<State> obstacle_pose_MtoA;

  // 回避位置生成と回避位置-自己位置間の障害物を検索するためのループ
  for (auto iter = 0; iter < NUM_ITERATIONS; iter++) {
    // 各変数を更新
    double distance = 0.0;  // 距離を格納する変数
    double distance_to_obstacle = 10000;  // 自己位置と障害物間の距離(適当な大きい値を格納)
    bool need_avoid = false;  // 障害物の存在の判定フラグ

    // 座標をロボット-目標位置間の座標系に変換
    tools::Trans trans_MtoA(my_robot_pose, tools::calc_angle(my_robot_pose, avoidance_pose));
    auto avoidance_pose_MtoA = trans_MtoA.transform(avoidance_pose);
    auto goal_pose_MtoA = trans_MtoA.transform(goal_pose);

    // ロボットに対して回避位置を生成
    for (const auto & robot : detection_tracked_->robots) {
      if (robot.visibility.size() == 0) {
        continue;
      }
      if (robot.visibility[0] < VISIBILITY_THRESHOLD) {
        continue;
      }

      // 自身の情報は除外する
      if (robot.robot_id.id == my_robot.robot_id.id &&
        robot.robot_id.team_color == my_robot.robot_id.team_color)
      {
        continue;
      }

      // ロボットが目標位置との間に存在するか判定
      auto robot_pose = tools::pose_state(robot);
      auto robot_pose_MtoA = trans_MtoA.transform(robot_pose);

      distance = std::hypot(robot_pose_MtoA.x, robot_pose_MtoA.y);
      if (0 < robot_pose_MtoA.x &&
        robot_pose_MtoA.x < avoidance_pose_MtoA.x &&
        std::fabs(robot_pose_MtoA.y) < OBSTACLE_DETECTION_Y_ROBOT)
      {
        if (distance < distance_to_obstacle) {
          obstacle_pose_MtoA = std::make_shared<State>(robot_pose_MtoA);
          distance_to_obstacle = distance;
          need_avoid = true;
        }
      }
    }

    // ボールが目標位置との間に存在するか判定
    if (avoid_ball) {
      auto ball_pose = tools::pose_state(ball);
      auto ball_pose_MtoA = trans_MtoA.transform(ball_pose);

      distance = std::hypot(ball_pose_MtoA.x, ball_pose_MtoA.y);
      // 進路上にボールエリアがある場合
      if (0 < ball_pose_MtoA.x &&
        ball_pose_MtoA.x < avoidance_pose_MtoA.x &&
        std::fabs(ball_pose_MtoA.y) < OBSTACLE_DETECTION_Y_BALL)
      {
        if (distance < distance_to_obstacle) {
          obstacle_pose_MtoA = std::make_shared<State>(ball_pose_MtoA);
          distance_to_obstacle = distance;
          need_avoid = true;
        }
      }
    }

    // 障害物が無い場合
    if (need_avoid == false) {
      // ループを抜ける
      break;
    } else {
      // 障害物がある場合
      // 相対的なY方向の回避位置を設定
      avoidance_pos_y = -std::copysign(AVOIDANCE_POS_Y, obstacle_pose_MtoA->y);
      // 障害物と距離が近い場合
      if (obstacle_pose_MtoA->x < OBSTACLE_DETECTION_X) {
        avoidance_pos_x = AVOIDANCE_POS_X_SHORT;
      } else {
        avoidance_pos_x = AVOIDANCE_POS_X_LONG;
      }

      // 回避位置を生成
      avoidance_pose = trans_MtoA.inverted_transform(
        obstacle_pose_MtoA->x + avoidance_pos_x,
        obstacle_pose_MtoA->y + avoidance_pos_y,
        goal_pose_MtoA.theta
      );
    }
  }
  return true;
}


bool FieldInfoParser::avoid_placement_area(
  const TrackedRobot & my_robot, const State & goal_pose, const TrackedBall & ball,
  const bool avoid_kick_receive_area,
  const State & designated_position, State & avoidance_pose) const
{
  // プレースメント範囲を回避する
  const double THRESHOLD_Y = 1.0;
  const double THRESHOLD_X = 0.65;
  const double AVOIDANCE_POS_X = 0.9;
  const double AVOIDANCE_POS_Y = 0.9;

  auto my_robot_pose = tools::pose_state(my_robot);
  auto ball_pose = tools::pose_state(ball);
  tools::Trans trans_BtoD(ball_pose, tools::calc_angle(ball_pose, designated_position));

  auto robot_pose_BtoD = trans_BtoD.transform(my_robot_pose);
  auto goal_pose_BtoD = trans_BtoD.transform(goal_pose);
  auto designated_BtoD = trans_BtoD.transform(designated_position);

  // 0.5 m 離れなければならない
  // 現在位置と目標位置がともにプレースメントエリアにある場合、回避点を生成する

  // 自チームのプレースメント時は、ボールを蹴る位置、受け取る位置を避けない
  double threshold_x = 0.0;
  if (avoid_kick_receive_area) {
    threshold_x = THRESHOLD_X;
  }
  bool my_pose_is_in_area = std::fabs(robot_pose_BtoD.y) < THRESHOLD_Y &&
    robot_pose_BtoD.x > -threshold_x &&
    robot_pose_BtoD.x < designated_BtoD.x + threshold_x;
  bool goal_pose_is_in_area = std::fabs(goal_pose_BtoD.y) < THRESHOLD_Y &&
    goal_pose_BtoD.x > -threshold_x &&
    goal_pose_BtoD.x < designated_BtoD.x + threshold_x;

  if (my_pose_is_in_area || goal_pose_is_in_area) {
    auto avoid_y = std::copysign(AVOIDANCE_POS_Y, robot_pose_BtoD.y);
    avoidance_pose = trans_BtoD.inverted_transform(robot_pose_BtoD.x, avoid_y, 0.0);

    // デッドロック回避
    const double FIELD_HALF_X = 6.0;
    const double FIELD_HALF_Y = 4.5;
    const double FIELD_WALL_X = 6.3;
    const double FIELD_WALL_Y = 4.8;

    const double FIELD_NEAR_WALL_X = FIELD_HALF_X - 0.0;
    const double FIELD_NEAR_WALL_Y = FIELD_HALF_Y - 0.0;
    auto avoid_x = std::copysign(AVOIDANCE_POS_X + 0.7, avoidance_pose.x);
    avoid_y = std::copysign(AVOIDANCE_POS_Y + 0.7, avoidance_pose.y);

    // フィールド外の場合，壁沿いに回避位置を生成
    if (FIELD_NEAR_WALL_Y < std::fabs(avoidance_pose.y)) {
      avoidance_pose.x = my_robot_pose.x - avoid_x;
    } else if (FIELD_NEAR_WALL_X < std::fabs(avoidance_pose.x)) {
      avoidance_pose.y = my_robot_pose.y - avoid_y;
    }

    // 壁の外に回避位置がある場合
    if (FIELD_WALL_Y < std::fabs(avoidance_pose.y)) {
      avoidance_pose.x = my_robot_pose.x;
      avoidance_pose.y = my_robot_pose.y - std::copysign(1.0, avoidance_pose.y);
    } else if (FIELD_WALL_X < std::fabs(avoidance_pose.x)) {
      avoidance_pose.x = my_robot_pose.x - std::copysign(1.0, avoidance_pose.x);
      avoidance_pose.y = my_robot_pose.y;
    }

    avoidance_pose.theta = my_robot_pose.theta;
  } else {
    avoidance_pose = goal_pose;
  }

  return true;
}

bool FieldInfoParser::avoid_robots(
  const TrackedRobot & my_robot, const State & goal_pose,
  State & avoidance_pose) const
{
  // ロボットを回避するposeを生成する
  // 全ロボット情報を検索し、
  // 目標位置とロボットが重なっている場合は、
  // 自己位置方向に目標位置をずらす

  const double ROBOT_DIAMETER = 0.18;  // meters ロボットの直径

  // ロボットを全探索
  for (const auto & robot : detection_tracked_->robots) {
    if (robot.visibility.size() == 0) {
      continue;
    }
    if (robot.visibility[0] < VISIBILITY_THRESHOLD) {
      continue;
    }

    // 自身の情報は除外する
    if (robot.robot_id.id == my_robot.robot_id.id &&
      robot.robot_id.team_color == my_robot.robot_id.team_color)
    {
      continue;
    }

    // ロボットの位置が目標位置上に存在するか判定
    auto robot_pose = tools::pose_state(robot);
    auto distance = tools::distance(robot_pose, goal_pose);
    if (distance < ROBOT_DIAMETER) {
      // 自己方向にずらした目標位置を生成
      auto my_robot_pose = tools::pose_state(my_robot);
      tools::Trans trans_GtoM(goal_pose, tools::calc_angle(goal_pose, my_robot_pose));
      avoidance_pose = trans_GtoM.inverted_transform(ROBOT_DIAMETER, 0.0, 0.0);
      avoidance_pose.theta = goal_pose.theta;
      return true;
    }
  }

  // 障害物がなければ、目標位置を回避位置とする
  avoidance_pose = goal_pose;
  return true;
}

bool FieldInfoParser::avoid_ball_500mm(
  const State & final_goal_pose,
  const State & goal_pose, const TrackedBall & ball,
  State & avoidance_pose) const
{
  // ボールから500 mm以上離れるために、回避処理を実行する
  // 目標位置がボールに近い場合はボールと目標位置の直線上で位置を離す
  // 回避後の目標位置がフィールド白線外部に生成された場合は、ボールの回避円周上で目標位置をずらす
  const double DISTANCE_TO_AVOID_THRESHOLD = 0.65;
  const double AVOID_MARGIN = 0.05;
  const double DISTANCE_TO_AVOID = DISTANCE_TO_AVOID_THRESHOLD - AVOID_MARGIN;

  auto ball_pose = tools::pose_state(ball);
  auto distance_BtoG = tools::distance(ball_pose, goal_pose);
  tools::Trans trans_BtoG(ball_pose, tools::calc_angle(ball_pose, goal_pose));
  auto final_goal_pose_BtoG = trans_BtoG.transform(final_goal_pose);

  // 障害物がなければ、目標位置を回避位置とする
  avoidance_pose = goal_pose;

  // 目標位置がボールに近づいている場合
  if (distance_BtoG < DISTANCE_TO_AVOID_THRESHOLD) {
    // 目標位置とボールを結ぶ直線上で、目標位置をボールから離す
    // このとき、最終目標位置側に回避位置を置く
    avoidance_pose = trans_BtoG.inverted_transform(
      std::copysign(DISTANCE_TO_AVOID, final_goal_pose_BtoG.x), 0.0, 0.0);
    avoidance_pose.theta = goal_pose.theta;

    if (!geometry_) {
      return true;
    }

    // フィールド外に目標位置が置かれた場合の処理
    const auto BOUNDARY_WIDTH = geometry_->field.boundary_width * 0.001;
    const auto FIELD_HALF_X = geometry_->field.field_length * 0.5 * 0.001;
    const auto FIELD_HALF_Y = geometry_->field.field_width * 0.5 * 0.001;
    // どれだけフィールドからはみ出たかを、0.0 ~ 1.0に変換する
    // はみ出た分だけ目標位置をボール周囲でずらす
    auto gain_x =
      std::clamp((std::fabs(avoidance_pose.x) - FIELD_HALF_X) / BOUNDARY_WIDTH, 0.0, 1.0);
    auto gain_y =
      std::clamp((std::fabs(avoidance_pose.y) - FIELD_HALF_Y) / BOUNDARY_WIDTH, 0.0, 1.0);
    if (gain_x > 0.0) {
      auto add_angle = std::copysign(gain_x * M_PI * 0.5, avoidance_pose.y);
      avoidance_pose = trans_BtoG.inverted_transform(
        DISTANCE_TO_AVOID * std::cos(add_angle),
        DISTANCE_TO_AVOID * std::sin(add_angle), 0.0);
    }
    if (gain_y > 0.0) {
      auto add_angle = std::copysign(gain_y * M_PI * 0.5, avoidance_pose.x);
      avoidance_pose = trans_BtoG.inverted_transform(
        DISTANCE_TO_AVOID * std::cos(add_angle),
        DISTANCE_TO_AVOID * std::sin(add_angle), 0.0);
    }
    avoidance_pose.theta = goal_pose.theta;
  }
  return true;
}

}  // namespace aisaac_consai_robot_controller

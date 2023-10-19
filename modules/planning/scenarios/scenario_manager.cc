/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/scenarios/scenario_manager.h"

#include <string>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
// #include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
// #include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"
// #include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {
namespace scenario {

// using apollo::hdmap::HDMapUtil;
// using apollo::hdmap::PathOverlap;

ScenarioManager::ScenarioManager(
    const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

bool ScenarioManager::Init(const PlanningConfig& planning_config) {
  planning_config_.CopyFrom(planning_config);
  RegisterScenarios();  // 注册lane follow场景，这个初始化函数应该只执行一次
  default_scenario_type_ = ScenarioConfig::LANE_FOLLOW;  // 默认是lane_follow
  current_scenario_ = CreateScenario(default_scenario_type_);  // 创建场景
  return true;
}

// 创建场景时，就把每个场景的stage注册下来了
std::unique_ptr<Scenario> ScenarioManager::CreateScenario(
    ScenarioConfig::ScenarioType scenario_type) {
  std::unique_ptr<Scenario> ptr;

  switch (scenario_type) {
    case ScenarioConfig::LANE_FOLLOW:
      ptr.reset(new lane_follow::LaneFollowScenario(    // 这里创建了具体lane_follow Scenario场景的实例，给了ptr
          config_map_[scenario_type], &scenario_context_, injector_)); // 函数末有个ptr->init()，调用的是基类Scenario里的init，基类里的init()有CreateStage()函数，
      break;                                                           // 是调用lane_follow_Scenario的CreateStage(继承了)，里边有注册stage和task
    default:
      return nullptr;
  }

  if (ptr != nullptr) {
    ptr->Init();  // 调用的是基类Scenario里的init，里边进行了stage注册
  }
  return ptr;
}

// TODO(all):目前只一种场景
void ScenarioManager::RegisterScenarios() {
  // lane_follow
  Scenario::LoadConfig(FLAGS_scenario_lane_follow_config_file, // FLAGS_scenario_lane_follow_config_file指向配置文件
                                &config_map_[ScenarioConfig::LANE_FOLLOW]);
  
}


// 用来决策当前处于什么场景，创建新的对象进行后续规划逻辑处理
void ScenarioManager::Update(const Frame& frame) {
  // ACHECK(!frame.reference_line_info().empty());

  // Observe(frame);  // 初始化第一次遇到重叠映射，不知道什么意思，像初始化

  ScenarioDispatch(frame);
}


// 决策场景，是用基于规则的方法还是学习的方法
void ScenarioManager::ScenarioDispatch(const Frame& frame) {
  // ACHECK(!frame.reference_line_info().empty());
  ScenarioConfig::ScenarioType scenario_type;
  scenario_type = ScenarioDispatchNonLearning(frame);

  std::cout << "select scenario: "
         << ScenarioConfig::ScenarioType_Name(scenario_type) << std::endl;

  // update PlanningContext
  // UpdatePlanningContext(frame, scenario_type);

  // 如果下一帧的场景和当前不同，则创建新场景
  if (current_scenario_->scenario_type() != scenario_type) {
    current_scenario_ = CreateScenario(scenario_type);
  }
}

ScenarioConfig::ScenarioType ScenarioManager::ScenarioDispatchLearning() {
  ////////////////////////////////////////
  // learning model scenario
  ScenarioConfig::ScenarioType scenario_type =
      ScenarioConfig::LEARNING_MODEL_SAMPLE;
  return scenario_type;
}

bool ScenarioManager::JudgeReachTargetPoint(
  const common::VehicleState& car_position,
  const common::PointENU& target_point) {
  double distance_to_vehicle =
    (car_position.x() - target_point.x()) *
    (car_position.x() - target_point.x()) +
    (car_position.y() - target_point.y()) *
    (car_position.y() - target_point.y());
  return distance_to_vehicle < FLAGS_threshold_distance_for_destination;
}

ScenarioConfig::ScenarioType ScenarioManager::ScenarioDispatchNonLearning(
    const Frame& frame) {
  ////////////////////////////////////////
  // default: LANE_FOLLOW
  ScenarioConfig::ScenarioType scenario_type = default_scenario_type_;
  // ////////////////////////////////////////
  // // Pad Msg scenario
  // scenario_type = SelectPadMsgScenario(frame);  // 根据驾驶员意图判断场景

  // // 如果不是默认场景，则直接输出；是的话，依次判断是否属于别的场景
  // const auto vehicle_state_provider = injector_->vehicle_state();  // injector_存放相关的信息
  // common::VehicleState vehicle_state = vehicle_state_provider->vehicle_state();
  // const common::PointENU& target_point =
  // frame.local_view().routing->routing_request().dead_end_info().target_point();
  // const common::VehicleState& car_position = frame.vehicle_state();
  // if (scenario_type == default_scenario_type_) { //检测到驾驶员想转换到lane follow场景
  //   // check current_scenario (not switchable)
  //   // switch函数case后不加break，后面的条件不管是否满足都会执行
  //   switch (current_scenario_->scenario_type()) {  //这里虽然写的是current_scenario_，但可以看做指的是上一帧的场景，因为要求换到驾驶员意图表达出的场景
  //     case ScenarioConfig::LANE_FOLLOW:
  //     case ScenarioConfig::PULL_OVER:
  //       break;
  //     case ScenarioConfig::BARE_INTERSECTION_UNPROTECTED:
  //     case ScenarioConfig::EMERGENCY_PULL_OVER:
  //     case ScenarioConfig::PARK_AND_GO:
  //     case ScenarioConfig::STOP_SIGN_PROTECTED:
  //     case ScenarioConfig::STOP_SIGN_UNPROTECTED:
  //     case ScenarioConfig::TRAFFIC_LIGHT_PROTECTED:
  //     case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
  //     case ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
  //     case ScenarioConfig::VALET_PARKING:
  //     case ScenarioConfig::DEADEND_TURNAROUND:
  //       // transfer dead_end to lane follow, should enhance transfer logic；dead_end转换到lane follow，应该加强转换逻辑
  //       if (JudgeReachTargetPoint(car_position, target_point)) {
  //         scenario_type = ScenarioConfig::LANE_FOLLOW;
  //         reach_target_pose_ = true;
  //       }
  //     case ScenarioConfig::YIELD_SIGN:
  //       // must continue until finish  如果上一个场景是让步场景，需要一直保持status到结束
  //       if (current_scenario_->GetStatus() !=
  //           Scenario::ScenarioStatus::STATUS_DONE) {
  //         scenario_type = current_scenario_->scenario_type();
  //       }
  //       break;
  //     default:
  //       break;
  //   }
  // }
  // ////////////////////////////////////////
  // // ParkAndGo / starting scenario    ；ParkAndGo猜测是停止后重新起步的场景
  // if (scenario_type == default_scenario_type_) {
  //   if (FLAGS_enable_scenario_park_and_go && !reach_target_pose_) {
  //     scenario_type = SelectParkAndGoScenario(frame);
  //   }
  // }

  // ////////////////////////////////////////
  // // intersection scenarios
  // if (scenario_type == default_scenario_type_) {
  //   scenario_type = SelectInterceptionScenario(frame);
  // }

  // ////////////////////////////////////////
  // // pull-over scenario
  // if (scenario_type == default_scenario_type_) {
  //   if (FLAGS_enable_scenario_pull_over) {
  //     scenario_type = SelectPullOverScenario(frame);
  //   }
  // }

  // ////////////////////////////////////////
  // // VALET_PARKING scenario
  // if (scenario_type == default_scenario_type_) {
  //   scenario_type = SelectValetParkingScenario(frame);
  // }
  // ////////////////////////////////////////
  // // dead end
  // if (scenario_type == default_scenario_type_) {
  //   scenario_type = SelectDeadEndScenario(frame);
  // }
  // ////////////////////////////////////////
  return scenario_type;  //驾驶员意图不是lane follow，则直接输出
}


}  // namespace scenario
}  // namespace planning
}  // namespace apollo

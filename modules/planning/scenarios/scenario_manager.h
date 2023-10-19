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

#pragma once

#include <memory>
#include <unordered_map>

// #include "modules/common/status/status.h"
// #include "modules/planning/common/planning_context.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {

class ScenarioManager final {
 public:
  ScenarioManager() = delete;

  explicit ScenarioManager(const std::shared_ptr<DependencyInjector>& injector);

  bool Init(const PlanningConfig& planning_config);

  Scenario* mutable_scenario() { return current_scenario_.get(); }

  DependencyInjector* injector() { return injector_.get(); }

  void Update(const Frame& frame);

 private:
  void Observe();

  std::unique_ptr<Scenario> CreateScenario(
      ScenarioConfig::ScenarioType scenario_type);

  void RegisterScenarios();

  void ScenarioDispatch(const Frame& frame);
  ScenarioConfig::ScenarioType ScenarioDispatchLearning();
  ScenarioConfig::ScenarioType ScenarioDispatchNonLearning(const Frame& frame);

  bool JudgeReachTargetPoint(const common::VehicleState& car_position,
                             const common::PointENU& target_point);

 private:
  std::shared_ptr<DependencyInjector> injector_;
  PlanningConfig planning_config_;
  std::unordered_map<ScenarioConfig::ScenarioType, ScenarioConfig,
                     std::hash<int>>
      config_map_;
  std::unique_ptr<Scenario> current_scenario_;
  ScenarioConfig::ScenarioType default_scenario_type_;
  ScenarioContext scenario_context_;
//   std::unordered_map<ReferenceLineInfo::OverlapType, hdmap::PathOverlap,
//                      std::hash<int>>
//       first_encountered_overlap_map_;
  bool routing_in_flag_ = true;
  common::PointENU dead_end_point_;
  bool reach_target_pose_ = false;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo

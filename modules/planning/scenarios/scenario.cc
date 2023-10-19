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

/**
 * @file
 **/

#include "modules/planning/scenarios/scenario.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/common/my_GetProtoFromFile/GetProtoFromFile.h"

namespace apollo {
namespace planning {
namespace scenario {

using apollo::planning::loadConfig;

Scenario::Scenario(const ScenarioConfig& config, const ScenarioContext* context,
                   const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), scenario_context_(context), injector_(injector) {
  name_ = ScenarioConfig::ScenarioType_Name(config.scenario_type());
}

bool Scenario::LoadConfig(const std::string& config_file,
                          ScenarioConfig* config) {
  loadConfig file_;
  return file_.GetProtoFromFile(config_file, config);
}

void Scenario::Init() {
  if(!config_.stage_type().empty()){
    // set scenario_type in PlanningContext
    auto* scenario = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_scenario();
    scenario->Clear();
    scenario->set_scenario_type(scenario_type());

    for (const auto& stage_config : config_.stage_config()) {
      // std::cout << stage_config.GetTypeName() << std::endl;
      stage_config_map_[stage_config.stage_type()] = &stage_config;
    }
    for (int i = 0; i < config_.stage_type_size(); ++i) {
      auto stage_type = config_.stage_type(i);
      // std::cout << stage_type << std::endl;
      if(!common::util::ContainsKey(stage_config_map_, stage_type)){
         std::cout << "stage type : " << ScenarioConfig::StageType_Name(stage_type)
          << " has no config" << std::endl;
      }
          
    }
    std::cout << "init stage "
          << ScenarioConfig::StageType_Name(config_.stage_type(0)) << std::endl;
    current_stage_ =
        CreateStage(*stage_config_map_[config_.stage_type(0)], injector_);  // 哪个类继承的Scenario，调用的就是那个类的CreateStage，注册stage
  } else{
     std::cout << scenario_type() << " config_.stage_type() is empty" << std::endl;
  }
  
}

Scenario::ScenarioStatus Scenario::Process(
    FrameHistory* frame_history, Frame* frame) {
  if (current_stage_ == nullptr) {
    std::cout << "Current stage is a null pointer." << std::endl;
    return STATUS_UNKNOWN;
  }
  if (current_stage_->stage_type() == ScenarioConfig::NO_STAGE) {
    scenario_status_ = STATUS_DONE;
    return scenario_status_;
  }
  auto ret = current_stage_->Process(frame_history, frame);
  switch (ret) {
    case Stage::ERROR: {
      std::cout << "Stage '" << current_stage_->Name() << "' returns error" << std::endl;
      scenario_status_ = STATUS_UNKNOWN;
      break;
    }
    case Stage::RUNNING: { // 正在处理
      scenario_status_ = STATUS_PROCESSING;
      break;
    }
    case Stage::FINISHED: {  //结束stage，看是否有下一个stage
      auto next_stage = current_stage_->NextStage();
      if (next_stage != current_stage_->stage_type()) {  //下一个stage和当前stage类型不同时
        std::cout << "switch stage from " << current_stage_->Name() << " to "
              << ScenarioConfig::StageType_Name(next_stage) << std::endl;
        if (next_stage == ScenarioConfig::NO_STAGE) {  // 所有stage都执行完了
          scenario_status_ = STATUS_DONE;
          return scenario_status_;
        }
        if (stage_config_map_.find(next_stage) == stage_config_map_.end()) {
          std::cout << "Failed to find config for stage: " << next_stage << std::endl;
          scenario_status_ = STATUS_UNKNOWN;
          return scenario_status_;
        }
        current_stage_ = CreateStage(*stage_config_map_[next_stage], injector_);  //创建新的stage
        if (current_stage_ == nullptr) {
          std::cout << "Current stage is a null pointer." << std::endl;
          return STATUS_UNKNOWN;
        }
      }
      if (current_stage_ != nullptr &&
          current_stage_->stage_type() != ScenarioConfig::NO_STAGE) {
        scenario_status_ = STATUS_PROCESSING;
      } else {
        scenario_status_ = STATUS_DONE;
      }
      break;
    }
    default: {
      std::cout << "Unexpected Stage return value: " << ret << std::endl;
      scenario_status_ = STATUS_UNKNOWN;
    }
  }
  return scenario_status_;
}

const std::string& Scenario::Name() const { return name_; }

}  // namespace scenario
}  // namespace planning
}  // namespace apollo

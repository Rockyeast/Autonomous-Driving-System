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

#include "modules/planning/scenarios/stage.h"

#include <unordered_map>
#include <utility>

#include "modules/planning/common/planning_context.h"
// #include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/tasks/task_factory.h"

namespace apollo {
namespace planning {
namespace scenario {

namespace {
// constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
// constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

// 构造函数里，根据stage注册所包含的task，存在task_list_中
Stage::Stage(const ScenarioConfig::StageConfig& config,
             const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), injector_(injector) {
  // set stage_type in PlanningContext
  injector->planning_context()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->set_stage_type(stage_type());

  name_ = ScenarioConfig::StageType_Name(config_.stage_type());
  next_stage_ = config_.stage_type();
  std::unordered_map<TaskConfig::TaskType, const TaskConfig*, std::hash<int>>
      config_map;
  for (const auto& task_config : config_.task_config()) {
    config_map[task_config.task_type()] = &task_config;
  }
  for (int i = 0; i < config_.task_type_size(); ++i) {
    auto task_type = config_.task_type(i);
    if(config_map.find(task_type) == config_map.end())
        std::cout << "Task: " << TaskConfig::TaskType_Name(task_type)
        << " used but not configured" << std::endl;
    auto iter = tasks_.find(task_type);
    if (iter == tasks_.end()) {
      auto ptr = TaskFactory::CreateTask(*config_map[task_type], injector_);
      task_list_.push_back(ptr.get());
      tasks_[task_type] = std::move(ptr);
    } else {
      task_list_.push_back(iter->second.get());
    }
  }
}

const std::string& Stage::Name() const { return name_; }

Task* Stage::FindTask(TaskConfig::TaskType task_type) const {
  auto iter = tasks_.find(task_type);
  if (iter == tasks_.end()) {
    return nullptr;
  } else {
    return iter->second.get();
  }
}

bool Stage::ExecuteTaskOnReferenceLine(
    FrameHistory* frame_history, Frame* frame) {
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (!reference_line_info.IsDrivable()) {
      std::cout << "The generated path is not drivable" << std::endl;
      return false;
    }

    for (auto* task : task_list_) {
      struct timeval start_timestamp, end_timestamp;  //记录规划时间
      gettimeofday(&start_timestamp, NULL);

      const auto ret = task->Execute(frame_history, frame, &reference_line_info);

      gettimeofday(&end_timestamp, NULL);
      long time_diff_us = (end_timestamp.tv_sec - start_timestamp.tv_sec) * 1000000 
                    + (end_timestamp.tv_usec - start_timestamp.tv_usec);
      std::cout << "after task[" << task->Name()
             << "]: " << reference_line_info.PathSpeedDebugString() << std::endl;
      std::cout << task->Name() << " time spend: " << time_diff_us << " us." << std::endl;
      RecordDebugInfo(&reference_line_info, task->Name(), time_diff_us);

      if (!ret) {
        std::cout << "Failed to run tasks[" << task->Name()
               << "], Error message: " << ret << std::endl;  // TODU(wushangzhe):eroor message
        break;
      }
    }

    // 规划失败没有速度路径时
    if (reference_line_info.speed_data().empty()) {
      // *reference_line_info.mutable_speed_data() =
      //     SpeedProfileGenerator::GenerateFallbackSpeed(injector_->ego_info());
      // reference_line_info.AddCost(kSpeedOptimizationFallbackCost);
      // reference_line_info.set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
    } else {
      // reference_line_info.set_trajectory_type(ADCTrajectory::NORMAL);
    }
    // 合并速度和路径曲线
    // DiscretizedTrajectory trajectory;
    // if (!reference_line_info.CombinePathAndSpeedProfile(
    //         planning_start_point.relative_time(),
    //         planning_start_point.path_point().s(), &trajectory)) {
    //   std::cout << "Fail to aggregate planning trajectory." << std::endl;
    //   return false;
    // }
    // reference_line_info.SetTrajectory(trajectory);
    // reference_line_info.SetDrivable(true);
    return true;
  }
  return true;
}


Stage::StageStatus Stage::FinishScenario() {
  next_stage_ = ScenarioConfig::NO_STAGE;
  return Stage::FINISHED;
}

void Stage::RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                            const std::string& name,
                            const double time_diff_ms) {
  // if (!FLAGS_enable_record_debug) {
  //   std::cout << "Skip record debug info" << std::endl;
  //   return;
  // }
  // if (reference_line_info == nullptr) {
  //   std::cout << "Reference line info is null." << std::endl;
  //   return;
  // }

  // auto ptr_latency_stats = reference_line_info->mutable_latency_stats();

  // auto ptr_stats = ptr_latency_stats->add_task_stats();
  // ptr_stats->set_name(name);
  // ptr_stats->set_time_ms(time_diff_ms);
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo

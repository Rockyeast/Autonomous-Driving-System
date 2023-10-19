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

#include "modules/planning/tasks/task_factory.h"

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"  // 一、添加新增task头文件
#include "modules/planning/tasks/deciders/st_generate_decider/st_generate_decider.h"
#include "modules/planning/tasks/deciders/new_test_decider/new_test_decider.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

apollo::common::util::Factory<
    TaskConfig::TaskType, Task,
    Task* (*)(const TaskConfig& config,
              const std::shared_ptr<DependencyInjector>& injector),
    std::unordered_map<
        TaskConfig::TaskType,
        Task* (*)(const TaskConfig& config,
                  const std::shared_ptr<DependencyInjector>& injector),
        std::hash<int>>>
    TaskFactory::task_factory_;

std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
    TaskFactory::default_task_configs_;

void TaskFactory::Init(const PlanningConfig& config,
                       const std::shared_ptr<DependencyInjector>& injector) {
  ///////////////////////////
  // deciders 注册所有决策器 task
  /**
   * 添加新task需要编写！！！
  */
  task_factory_.Register(
      TaskConfig::LANE_CHANGE_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new LaneChangeDecider(config, injector); // 填写自己编写的cc、h文件中的类名
      });
  
  task_factory_.Register(
      TaskConfig::NEW_TEST_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new NewTestDecider(config, injector);
      });
  
  task_factory_.Register(
      TaskConfig::ST_GENERATE_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new StGenerateDecider(config, injector);
      });

  // task_factory_.Register(
  //     TaskConfig::OPEN_SPACE_FALLBACK_DECIDER,
  //     [](const TaskConfig& config,
  //        const std::shared_ptr<DependencyInjector>& injector) -> Task* {
  //       return new OpenSpaceFallbackDecider(config, injector);
  //     });

  ///////////////////////////
  // optimizers 注册所有优化器task
  // task_factory_.Register(
  //     TaskConfig::OPEN_SPACE_TRAJECTORY_PARTITION,
  //     [](const TaskConfig& config,
  //        const std::shared_ptr<DependencyInjector>& injector) -> Task* {
  //       return new OpenSpaceTrajectoryPartition(config, injector);
  //     });
  // task_factory_.Register(
  //     TaskConfig::OPEN_SPACE_TRAJECTORY_PROVIDER,
  //     [](const TaskConfig& config,
  //        const std::shared_ptr<DependencyInjector>& injector) -> Task* {
  //       return new OpenSpaceTrajectoryProvider(config, injector);
  //     });
  ///////////////////////////
  // other tasks
  // task_factory_.Register(
  //     TaskConfig::LEARNING_MODEL_INFERENCE_TASK,
  //     [](const TaskConfig& config,
  //        const std::shared_ptr<DependencyInjector>& injector) -> Task* {
  //       return new LearningModelInferenceTask(config, injector);
  //     });
  // task_factory_.Register(
  //     TaskConfig::LEARNING_MODEL_INFERENCE_TRAJECTORY_TASK,
  //     [](const TaskConfig& config,
  //        const std::shared_ptr<DependencyInjector>& injector) -> Task* {
  //       return new LearningModelInferenceTrajectoryTask(config, injector);
  //     });

  for (const auto& default_task_config : config.default_task_config()) {
    default_task_configs_[default_task_config.task_type()] =
        default_task_config;
  }
}

std::unique_ptr<Task> TaskFactory::CreateTask(
    const TaskConfig& task_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  TaskConfig merged_config;
  // 打印的是主配置文件中default_task_configs的数量，可以根据这个判断配置文件的读取是否正常
  // std::cout << "default_task_configs_.size() = " << default_task_configs_.size() << std::endl; 
  if (default_task_configs_.find(task_config.task_type()) !=
      default_task_configs_.end()) {
    merged_config = default_task_configs_[task_config.task_type()];
  }
  merged_config.MergeFrom(task_config);
  return task_factory_.CreateObject(task_config.task_type(), merged_config,
                                    injector);
}

}  // namespace planning
}  // namespace apollo

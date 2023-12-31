/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

// #include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/history.h"
// #include "modules/planning/common/learning_based_data.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

//将planning所有涉及到的相关信息都集中存放到这个类里
class DependencyInjector {
 public:
  DependencyInjector() = default;
  ~DependencyInjector() = default;

  //planning_context_是PlanningContext类
  //该成员存放planning的状态PlanningStatus
  //planning的状态包括：bare_intersection无保护路口、change_lane变道、
  //creep_decider缓行决策等等
  PlanningContext* planning_context() {
    return &planning_context_;
  }

  //Frame保存一个规划周期的所有数据，如定位/底盘，道路参考线信息，车辆状态，规划起始点等
  //FrameHistory存放历史的frame信息，默认只存一帧历史信息。
  // FrameHistory* frame_history() {
  //   return &frame_history_;
  // }

  //history_存放历史frame信息以及历史障碍物状态信息？例如绕行，动静态障碍物等
  // History* history() {
  //   return &history_;
  // }

  //ego_info_存放车辆自身状态信息包括一些周围环境信息，自车box盒，vehicle_state
  //以及轨迹拼接点/自车位置等信息
  EgoInfo* ego_info() {
    return &ego_info_;
  }

  //车辆状态信息
  //定位以及底盘反馈的一些量
  // apollo::common::VehicleStateProvider* vehicle_state() {
  //   return &vehicle_state_;
  // }

  //基于学习决策，现阶段先略过，后续掌握传统planning后可以学习下
  // LearningBasedData* learning_based_data() {
  //   return &learning_based_data_;
  // }

 private:
  PlanningContext planning_context_;
  // FrameHistory frame_history_;
  // History history_;
  EgoInfo ego_info_;
  // apollo::common::VehicleStateProvider vehicle_state_;
  // LearningBasedData learning_based_data_;
};

}  // namespace planning
}  // namespace apollo

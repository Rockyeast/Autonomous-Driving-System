/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"

#include <limits>
#include <memory>

#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;

LaneChangeDecider::LaneChangeDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  // TODO(wushangzhe):缺少对配置文件打开安全检查
  // ACHECK(config_.has_lane_change_decider_config());
}

// 每个task处理入口
bool LaneChangeDecider::Process(FrameHistory* frame_history, 
    Frame* frame, ReferenceLineInfo* const current_reference_line_info) {

  // 按以下步骤进行task编写

  // 1.合理性检查：检查传入参数是否为空

  // 2.获取配置文件参数、frame中所需数据（同时进行合理性检查）
  
  // 3.逻辑代码编写。（建议大块逻辑代码放到Process之外处理，或者另开文件类进行处理，保持Process简洁易读）

  // 4.运行成功：结果保存至frame对应位置，return true
  //   运行失败: 相应处理措施，输出失败原因，return false

  // 样例：
  const auto& lane_change_decider_config = config_.lane_change_decider_config();  // modules/planning/conf/planning_config.pb.txt

  std::list<ReferenceLineInfo>* reference_line_info =
      frame->mutable_reference_line_info();
  if (reference_line_info->empty()) {
    const std::string msg = "Reference lines empty.";
    std::cout << msg;
    return false;
  }

  
  return true;
}


}  // namespace planning
}  // namespace apollo

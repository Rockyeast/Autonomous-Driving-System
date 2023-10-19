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

/**
 * @file
 **/

#pragma once

#include <list>
#include <memory>
#include <string>

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_status.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class NewTestDecider : public Decider {
 public:
  NewTestDecider(const TaskConfig& config,
                    const std::shared_ptr<DependencyInjector>& injector);
 private:
  bool Process(
      FrameHistory* frame_history, Frame* frame,
      ReferenceLineInfo* const current_reference_line_info) override;

  bool testFunction(Frame* frame, ReferenceLineInfo* const current_reference_line_info);
};

}  // namespace planning
}  // namespace apollo

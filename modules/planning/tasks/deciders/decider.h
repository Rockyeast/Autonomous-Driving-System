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

#pragma once

#include <memory>

#include "modules/planning/common/frame.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

class Decider : public Task {
 public:
  explicit Decider(const TaskConfig& config);
  Decider(const TaskConfig& config,
          const std::shared_ptr<DependencyInjector>& injector);
  virtual ~Decider() = default;

  bool Execute(FrameHistory* frame_history, Frame* frame, 
               ReferenceLineInfo* reference_line_info) override;

  bool Execute(Frame* frame) override;

 protected:
  virtual bool Process(
      FrameHistory* frame_history, Frame* frame, ReferenceLineInfo* reference_line_info) {
    return true;
  }

  virtual bool Process(Frame* frame) {
    return true;
  }
};

}  // namespace planning
}  // namespace apollo

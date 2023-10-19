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
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/math/box2d.h"
#include "modules/Debug/Debug_Interface.h"



namespace apollo {
namespace planning {

using apollo::common::VehicleParam;
using apollo::common::math::Box2d;

class StGenerateDecider : public Decider {
 public:
  StGenerateDecider(const TaskConfig& config,
                    const std::shared_ptr<DependencyInjector>& injector);
 private:
  bool Process(
      FrameHistory* frame_history, Frame* frame,
      ReferenceLineInfo* const current_reference_line_info) override;

  bool CreatSTGraphByParallelogram(Frame* frame, PathData path_data, StGenerateDeciderConfig config);
  bool CreatSTGraphByLine(Frame* frame, PathData path_data);
  Box2d GetADCBoundingBox(const PathPoint& path_point, const VehicleParam& vehicle_param, double& buffer);
  bool CheckCollision(Box2d& adc_box, const Box2d& obs_box);
  void DebugInfo_st(Frame* frame);
  void DebugInfo_trj(Frame* frame, PathData path_data);
  bool CalculateSpeedLimit(StGenerateDeciderConfig st_generate_decider_config, const std::vector<Obstacle *> obstacles, 
                ReferenceLineInfo* current_reference_line_info, PathData path_data);

  // back使用，后期删除
  bool CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double l_buffer) const;
};

}  // namespace planning
}  // namespace apollo

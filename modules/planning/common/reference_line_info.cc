/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/common/reference_line_info.h"

#include <algorithm>

#include "absl/strings/str_cat.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/util.h"
#include "modules/planning/proto/planning_status.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"

namespace apollo {
namespace planning {

using apollo::canbus::Chassis;
using apollo::common::EngageAdvice;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleSignal;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::util::PointFactory;

std::unordered_map<std::string, bool>
    ReferenceLineInfo::junction_right_of_way_map_;

ReferenceLineInfo::ReferenceLineInfo(const common::VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line){}


const std::vector<PathData>& ReferenceLineInfo::GetCandidatePathData() const {
  return candidate_path_data_;
}

void ReferenceLineInfo::SetCandidatePathData(
    std::vector<PathData>&& candidate_path_data) {
  candidate_path_data_ = std::move(candidate_path_data);
}

const std::vector<PathBoundary>& ReferenceLineInfo::GetCandidatePathBoundaries()
    const {
  return candidate_path_boundaries_;
}

void ReferenceLineInfo::SetCandidatePathBoundaries(
    std::vector<PathBoundary>&& path_boundaries) {
  candidate_path_boundaries_ = std::move(path_boundaries);
}

double ReferenceLineInfo::GetCruiseSpeed() const {
  return cruise_speed_ > 0.0 ? cruise_speed_ : FLAGS_default_cruise_speed;
}

// bool ReferenceLineInfo::GetNeighborLaneInfo(
//     const ReferenceLineInfo::LaneType lane_type, const double s,
//     hdmap::Id* ptr_lane_id, double* ptr_lane_width) const {
//   auto ptr_lane_info = LocateLaneInfo(s);
//   if (ptr_lane_info == nullptr) {
//     return false;
//   }

//   switch (lane_type) {
//     case LaneType::LeftForward: {
//       if (ptr_lane_info->lane().left_neighbor_forward_lane_id().empty()) {
//         return false;
//       }
//       *ptr_lane_id = ptr_lane_info->lane().left_neighbor_forward_lane_id(0);
//       break;
//     }
//     case LaneType::LeftReverse: {
//       if (ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
//         return false;
//       }
//       *ptr_lane_id = ptr_lane_info->lane().left_neighbor_reverse_lane_id(0);
//       break;
//     }
//     case LaneType::RightForward: {
//       if (ptr_lane_info->lane().right_neighbor_forward_lane_id().empty()) {
//         return false;
//       }
//       *ptr_lane_id = ptr_lane_info->lane().right_neighbor_forward_lane_id(0);
//       break;
//     }
//     case LaneType::RightReverse: {
//       if (ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
//         return false;
//       }
//       *ptr_lane_id = ptr_lane_info->lane().right_neighbor_reverse_lane_id(0);
//       break;
//     }
//     default:
//       ACHECK(false);
//   }
//   auto ptr_neighbor_lane =
//       hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(*ptr_lane_id);
//   if (ptr_neighbor_lane == nullptr) {
//     return false;
//   }

//   auto ref_point = reference_line_.GetReferencePoint(s);

//   double neighbor_s = 0.0;
//   double neighbor_l = 0.0;
//   if (!ptr_neighbor_lane->GetProjection({ref_point.x(), ref_point.y()},
//                                         &neighbor_s, &neighbor_l)) {
//     return false;
//   }

//   *ptr_lane_width = ptr_neighbor_lane->GetWidth(neighbor_s);
//   return true;
// }





ADCTrajectory::RightOfWayStatus ReferenceLineInfo::GetRightOfWayStatus() const {
  
}


const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}

ReferenceLine* ReferenceLineInfo::mutable_reference_line() {
  return &reference_line_;
}

void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

// bool ReferenceLineInfo::AddObstacleHelper(
//     const std::shared_ptr<Obstacle>& obstacle) {
//   return AddObstacle(obstacle.get()) != nullptr;
// }

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}

// bool ReferenceLineInfo::IsStartFrom(
//     const ReferenceLineInfo& previous_reference_line_info) const {
//   if (reference_line_.reference_points().empty()) {
//     return false;
//   }
//   auto start_point = reference_line_.reference_points().front();
//   const auto& prev_reference_line =
//       previous_reference_line_info.reference_line();
//   common::SLPoint sl_point;
//   prev_reference_line.XYToSL(start_point, &sl_point);
//   return previous_reference_line_info.reference_line_.IsOnLane(sl_point);
// }

const SpeedLimit& ReferenceLineInfo::optimize_speed_limit() const { return optimize_speed_limit_; }

const PathData& ReferenceLineInfo::path_data() const { return path_data_; }

const PathData& ReferenceLineInfo::fallback_path_data() const {
  return fallback_path_data_;
}

const SpeedData& ReferenceLineInfo::speed_data() const { return speed_data_; }

PathData* ReferenceLineInfo::mutable_path_data() { return &path_data_; }

PathData* ReferenceLineInfo::mutable_fallback_path_data() {
  return &fallback_path_data_;
}

SpeedLimit* ReferenceLineInfo::mutable_optimize_speed_limit() {
  return &optimize_speed_limit_;
}

SpeedData* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }

bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  // ACHECK(ptr_discretized_trajectory != nullptr);
  // // use varied resolution to reduce data load but also provide enough data
  // // point for control module
  // const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
  // const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
  // const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;

  // if (path_data_.discretized_path().empty()) {
  //   AERROR << "path data is empty";
  //   return false;
  // }

  // if (speed_data_.empty()) {
  //   AERROR << "speed profile is empty";
  //   return false;
  // }

  // for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();
  //      cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
  //                                                    : kSparseTimeResolution)) {
  //   common::SpeedPoint speed_point;
  //   if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
  //     AERROR << "Fail to get speed point with relative time " << cur_rel_time;
  //     return false;
  //   }

  //   if (speed_point.s() > path_data_.discretized_path().Length()) {
  //     break;
  //   }
  //   common::PathPoint path_point =
  //       path_data_.GetPathPointWithPathS(speed_point.s());
  //   path_point.set_s(path_point.s() + start_s);

  //   common::TrajectoryPoint trajectory_point;
  //   trajectory_point.mutable_path_point()->CopyFrom(path_point);
  //   trajectory_point.set_v(speed_point.v());
  //   trajectory_point.set_a(speed_point.a());
  //   trajectory_point.set_relative_time(speed_point.t() + relative_time);
  //   ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  // }
  // return true;
}

// bool ReferenceLineInfo::AdjustTrajectoryWhichStartsFromCurrentPos(
//     const common::TrajectoryPoint& planning_start_point,
//     const std::vector<common::TrajectoryPoint>& trajectory,
//     DiscretizedTrajectory* adjusted_trajectory) {
//   ACHECK(adjusted_trajectory != nullptr);
//   // find insert index by check heading
//   static constexpr double kMaxAngleDiff = M_PI_2;
//   const double start_point_heading = planning_start_point.path_point().theta();
//   const double start_point_x = planning_start_point.path_point().x();
//   const double start_point_y = planning_start_point.path_point().y();
//   const double start_point_relative_time = planning_start_point.relative_time();

//   int insert_idx = -1;
//   for (size_t i = 0; i < trajectory.size(); ++i) {
//     // skip trajectory_points early than planning_start_point
//     if (trajectory[i].relative_time() <= start_point_relative_time) {
//       continue;
//     }

//     const double cur_point_x = trajectory[i].path_point().x();
//     const double cur_point_y = trajectory[i].path_point().y();
//     const double tracking_heading =
//         std::atan2(cur_point_y - start_point_y, cur_point_x - start_point_x);
//     if (std::fabs(common::math::AngleDiff(start_point_heading,
//                                           tracking_heading)) < kMaxAngleDiff) {
//       insert_idx = i;
//       break;
//     }
//   }
//   if (insert_idx == -1) {
//     AERROR << "All points are behind of planning init point";
//     return false;
//   }

//   DiscretizedTrajectory cut_trajectory(trajectory);
//   cut_trajectory.erase(cut_trajectory.begin(),
//                        cut_trajectory.begin() + insert_idx);
//   cut_trajectory.insert(cut_trajectory.begin(), planning_start_point);

//   // In class TrajectoryStitcher, the stitched point which is also the planning
//   // init point is supposed have one planning_cycle_time ahead respect to
//   // current timestamp as its relative time. So the relative timelines
//   // of planning init point and the trajectory which start from current
//   // position(relative time = 0) are the same. Therefore any conflicts on the
//   // relative time including the one below should return false and inspected its
//   // cause.
//   if (cut_trajectory.size() > 1 && cut_trajectory.front().relative_time() >=
//                                        cut_trajectory[1].relative_time()) {
//     AERROR << "planning init point relative_time["
//            << cut_trajectory.front().relative_time()
//            << "] larger than its next point's relative_time["
//            << cut_trajectory[1].relative_time() << "]";
//     return false;
//   }

//   // In class TrajectoryStitcher, the planing_init_point is set to have s as 0,
//   // so adjustment is needed to be done on the other points
//   double accumulated_s = 0.0;
//   for (size_t i = 1; i < cut_trajectory.size(); ++i) {
//     const auto& pre_path_point = cut_trajectory[i - 1].path_point();
//     auto* cur_path_point = cut_trajectory[i].mutable_path_point();
//     accumulated_s += std::sqrt((cur_path_point->x() - pre_path_point.x()) *
//                                    (cur_path_point->x() - pre_path_point.x()) +
//                                (cur_path_point->y() - pre_path_point.y()) *
//                                    (cur_path_point->y() - pre_path_point.y()));
//     cur_path_point->set_s(accumulated_s);
//   }

//   // reevaluate relative_time to make delta t the same
//   adjusted_trajectory->clear();
//   // use varied resolution to reduce data load but also provide enough data
//   // point for control module
//   const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
//   const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
//   const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;
//   for (double cur_rel_time = cut_trajectory.front().relative_time();
//        cur_rel_time <= cut_trajectory.back().relative_time();
//        cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
//                                                      : kSparseTimeResolution)) {
//     adjusted_trajectory->AppendTrajectoryPoint(
//         cut_trajectory.Evaluate(cur_rel_time));
//   }
//   return true;
// }

void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }

bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }

// 判断是否为可变车道，判断自车是否在当前车道片段上（不在则为true）
bool ReferenceLineInfo::IsChangeLanePath() const {
  // return !Lanes().IsOnSegment();
}

bool ReferenceLineInfo::IsNeighborLanePath() const {
  // return Lanes().IsNeighborSegment();
}

std::string ReferenceLineInfo::PathSpeedDebugString() const {
  return absl::StrCat("path_data:", path_data_.DebugString(),
                      "speed_data:", speed_data_.DebugString());
}

}  // namespace planning
}  // namespace apollo

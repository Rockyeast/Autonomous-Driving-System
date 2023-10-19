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

#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"

#include <utility>

#include "modules/common/math/math_utils.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
// #include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"  // lane_follow中要包含的task头文件写在下边

namespace apollo {
namespace planning {
namespace scenario {
namespace lane_follow {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::util::PointFactory;

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

//构造函数里调用了stage的构造函数，注册所包含的task
LaneFollowStage::LaneFollowStage(
    const ScenarioConfig::StageConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Stage(config, injector) {}

void LaneFollowStage::RecordObstacleDebugInfo(
    ReferenceLineInfo* reference_line_info) {
  // if (!FLAGS_enable_record_debug) {
  //   std::cout << "Skip record debug info" << std::endl;
  //   return;
  // }
  // auto ptr_debug = reference_line_info->mutable_debug();

  // const auto path_decision = reference_line_info->path_decision();
  // for (const auto obstacle : path_decision->obstacles().Items()) {
  //   auto obstacle_debug = ptr_debug->mutable_planning_data()->add_obstacle();
  //   obstacle_debug->set_id(obstacle->Id());
  //   obstacle_debug->mutable_sl_boundary()->CopyFrom(
  //       obstacle->PerceptionSLBoundary());
  //   const auto& decider_tags = obstacle->decider_tags();
  //   const auto& decisions = obstacle->decisions();
  //   if (decider_tags.size() != decisions.size()) {
  //     std::cout << "decider_tags size: " << decider_tags.size()
  //            << " different from decisions size:" << decisions.size() << std::endl;
  //   }
  //   for (size_t i = 0; i < decider_tags.size(); ++i) {
  //     auto decision_tag = obstacle_debug->add_decision_tag();
  //     decision_tag->set_decider_tag(decider_tags[i]);
  //     decision_tag->mutable_decision()->CopyFrom(decisions[i]);
  //   }
  // }
}

Stage::StageStatus LaneFollowStage::Process(
    FrameHistory* frame_history, Frame* frame) {
  bool has_drivable_reference_line = false;

  std::cout << "Number of reference lines:\t"
         << frame->mutable_reference_line_info()->size() << std::endl;

  unsigned int count = 0;

  // 遍历所有参考线，直到找到可用的
  // TODU(wushangzhe):这里就要找出要变道的参考线了
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    // TODO(SHU): need refactor
    if (count++ == frame->mutable_reference_line_info()->size()) {
      break;
    }
    std::cout << "No: [" << count << "] Reference Line." << std::endl;
    // std::cout << "IsChangeLanePath: " << reference_line_info.IsChangeLanePath() << std::endl;  // 打印是否是变道参考线

    // 找到了，退出
    if (has_drivable_reference_line) {
      reference_line_info.SetDrivable(false);
      break;
    }

    // 主逻辑
    auto cur_status =
        PlanOnReferenceLine(frame_history, frame, &reference_line_info);

    if (cur_status) {

      // TODO(wushangzhe): ! 这里要重写，因为只有一个stage，处理完了轨迹就有了，可以在这里考虑和控制的交互和赋值上一帧信息

      // if (reference_line_info.IsChangeLanePath()) {
      //   // 如果发生了lanechange，判断reference_line的cost
      //   std::cout << "reference line is lane change ref." << std::endl;
      //   std::cout << "FLAGS_enable_smarter_lane_change: "
      //          << FLAGS_enable_smarter_lane_change << std::endl;
      //   if (reference_line_info.Cost() < kStraightForwardLineCost &&            // 不变道的cost == kStraightForwardLineCost
      //       (LaneChangeDecider::IsClearToChangeLane(&reference_line_info) ||
      //        FLAGS_enable_smarter_lane_change)) {
      //     //在智能变道或IsClearToChangeLane模式下速度和路径优化成功的话
      //     // If the path and speed optimization succeed on target lane while
      //     // under smart lane-change or IsClearToChangeLane under older version
      //     has_drivable_reference_line = true;
      //     reference_line_info.SetDrivable(true);
      //     LaneChangeDecider::UpdatePreparationDistance(
      //         true, frame, &reference_line_info, injector_->planning_context());
      //     std::cout << "\tclear for lane change" << std::endl;
      //   } else {
      //     LaneChangeDecider::UpdatePreparationDistance(
      //         false, frame, &reference_line_info,
      //         injector_->planning_context());
      //     reference_line_info.SetDrivable(false);
      //     std::cout << "\tlane change failed" << std::endl;
      //   }
      // } else {
      //   // 如果规划成功，并且没有变道，则has_drivable_reference_line为true
      //   std::cout << "reference line is NOT lane change ref." << std::endl;
      //   has_drivable_reference_line = true;
      // }
    } else {
      // TODO(wushangzhe)：！失败处理
      // reference_line_info.SetDrivable(false);
    }
  }

  return StageStatus::FINISHED;
  // return has_drivable_reference_line ? StageStatus::RUNNING
  //                                   : StageStatus::ERROR;
}

bool LaneFollowStage::PlanOnReferenceLine(
    FrameHistory* frame_history, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->IsChangeLanePath()) {
    reference_line_info->AddCost(kStraightForwardLineCost);
  }
  // std::cout << "planning start point:" << planning_start_point.DebugString() << std::endl;
  // std::cout << "Current reference_line_info is IsChangeLanePath: "
  //        << reference_line_info->IsChangeLanePath() << std::endl;

  auto ret = true;
  // 执行每一个task
  for (auto* task : task_list_) {
    struct timeval start_timestamp, end_timestamp;  // 计算花费时间
    gettimeofday(&start_timestamp, NULL);

    ret = task->Execute(frame_history, frame, reference_line_info);

    gettimeofday(&end_timestamp, NULL);
    long time_diff_us = (end_timestamp.tv_sec - start_timestamp.tv_sec) * 1000000 
                  + (end_timestamp.tv_usec - start_timestamp.tv_usec);
    // 每一个task过后打印一下路径规划和速度规划的结果
    // std::cout << "after task[" << task->Name()
    //        << "]:" << reference_line_info->PathSpeedDebugString();
    std::cout << task->Name() << " time spend: " << time_diff_us << " us." << std::endl << std::endl;
    RecordDebugInfo(reference_line_info, task->Name(), time_diff_us);

    if (!ret) {
      std::cout << "Failed to run tasks[" << task->Name()
             << "], Error message: " << ret << std::endl; 
      break;
    }
  }

  // TODO(wushangzhe)：！需要重写，规划完成对frame中结果的赋值 && 失败处理

  // //记录障碍物debug信息（不清楚干嘛的）
  // RecordObstacleDebugInfo(reference_line_info);

  // // check path and speed results for path or speed fallback 检查路径和速度结果是否失败
  // reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  // if (!ret) {
  //   PlanFallbackTrajectory(planning_start_point, frame, reference_line_info);
  // }

  // DiscretizedTrajectory trajectory;
  // if (!reference_line_info->CombinePathAndSpeedProfile(  // 合并速度和路径曲线
  //         planning_start_point.relative_time(),
  //         planning_start_point.path_point().s(), &trajectory)) {
  //   const std::string msg = "Fail to aggregate planning trajectory.";
  //   std::cout << msg << std::endl;
  //   return false;
  // }

  // // determine if there is a destination on reference line. 确定参考线上是否有目的地
  // double dest_stop_s = -1.0;
  // for (const auto* obstacle :
  //      reference_line_info->path_decision()->obstacles().Items()) {
  //   if (obstacle->LongitudinalDecision().has_stop() &&
  //       obstacle->LongitudinalDecision().stop().reason_code() ==
  //           STOP_REASON_DESTINATION) {
  //     SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
  //                                 reference_line_info->reference_line());
  //     dest_stop_s = dest_sl.s();
  //   }
  // }

  // for (const auto* obstacle :
  //      reference_line_info->path_decision()->obstacles().Items()) {
  //   if (obstacle->IsVirtual()) {
  //     continue;
  //   }
  //   if (!obstacle->IsStatic()) {
  //     continue;
  //   }
  //   if (obstacle->LongitudinalDecision().has_stop()) {
  //     bool add_stop_obstacle_cost = false;
  //     if (dest_stop_s < 0.0) {
  //       add_stop_obstacle_cost = true;
  //     } else {
  //       SLPoint stop_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
  //                                   reference_line_info->reference_line());
  //       if (stop_sl.s() < dest_stop_s) {
  //         add_stop_obstacle_cost = true;
  //       }
  //     }
  //     if (add_stop_obstacle_cost) {
  //       static constexpr double kReferenceLineStaticObsCost = 1e3;
  //       reference_line_info->AddCost(kReferenceLineStaticObsCost);
  //     }
  //   }
  // }

  // if (FLAGS_enable_trajectory_check) {
  //   if (ConstraintChecker::ValidTrajectory(trajectory) !=
  //       ConstraintChecker::Result::VALID) {
  //     const std::string msg = "Current planning trajectory is not valid.";
  //     std::cout << msg << std::endl;
  //     return false;
  //   }
  // }

  // reference_line_info->SetTrajectory(trajectory);
  // reference_line_info->SetDrivable(true);
  return true;
}

// void LaneFollowStage::PlanFallbackTrajectory(
//     const TrajectoryPoint& planning_start_point, Frame* frame,
//     ReferenceLineInfo* reference_line_info) {
//   // path and speed fall back
//   if (reference_line_info->path_data().Empty()) {
//     std::cout << "Path fallback due to algorithm failure" << std::endl;
//     GenerateFallbackPathProfile(reference_line_info,
//                                 reference_line_info->mutable_path_data());
//     reference_line_info->AddCost(kPathOptimizationFallbackCost);
//     reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
//   }

//   if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) {
//     if (!RetrieveLastFramePathProfile(
//             reference_line_info, frame,
//             reference_line_info->mutable_path_data())) {
//       const auto& candidate_path_data =
//           reference_line_info->GetCandidatePathData();
//       for (const auto& path_data : candidate_path_data) {
//         if (path_data.path_label().find("self") != std::string::npos) {
//           *reference_line_info->mutable_path_data() = path_data;
//           std::cout << "Use current frame self lane path as fallback " << std::endl;
//           break;
//         }
//       }
//     }
//   }

//   std::cout << "Speed fallback due to algorithm failure" << std::endl;
//   *reference_line_info->mutable_speed_data() =
//       SpeedProfileGenerator::GenerateFallbackSpeed(injector_->ego_info());

//   if (reference_line_info->trajectory_type() != ADCTrajectory::PATH_FALLBACK) {
//     reference_line_info->AddCost(kSpeedOptimizationFallbackCost);
//     reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
//   }
// }

// void LaneFollowStage::GenerateFallbackPathProfile(
//     const ReferenceLineInfo* reference_line_info, PathData* path_data) {
//   const double unit_s = 1.0;
//   const auto& reference_line = reference_line_info->reference_line();

//   auto adc_point = injector_->ego_info()->start_point();
//   DCHECK(adc_point.has_path_point());
//   const auto adc_point_x = adc_point.path_point().x();
//   const auto adc_point_y = adc_point.path_point().y();

//   common::SLPoint adc_point_s_l;
//   if (!reference_line.XYToSL(adc_point.path_point(), &adc_point_s_l)) {
//     std::cout << "Fail to project ADC to reference line when calculating path "
//               "fallback. Straight forward path is generated" << std::endl;
//     const auto adc_point_heading = adc_point.path_point().theta();
//     const auto adc_point_kappa = adc_point.path_point().kappa();
//     const auto adc_point_dkappa = adc_point.path_point().dkappa();
//     std::vector<common::PathPoint> path_points;
//     double adc_traversed_x = adc_point_x;
//     double adc_traversed_y = adc_point_y;

//     const double max_s = 100.0;
//     for (double s = 0; s < max_s; s += unit_s) {
//       path_points.push_back(PointFactory::ToPathPoint(
//           adc_traversed_x, adc_traversed_y, 0.0, s, adc_point_heading,
//           adc_point_kappa, adc_point_dkappa));
//       adc_traversed_x += unit_s * std::cos(adc_point_heading);
//       adc_traversed_y += unit_s * std::sin(adc_point_heading);
//     }
//     path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
//     return;
//   }

//   // Generate a fallback path along the reference line direction
//   const auto adc_s = adc_point_s_l.s();
//   const auto& adc_ref_point =
//       reference_line.GetReferencePoint(adc_point_x, adc_point_y);
//   const double dx = adc_point_x - adc_ref_point.x();
//   const double dy = adc_point_y - adc_ref_point.y();

//   std::vector<common::PathPoint> path_points;
//   const double max_s = reference_line.Length();
//   for (double s = adc_s; s < max_s; s += unit_s) {
//     const auto& ref_point = reference_line.GetReferencePoint(s);
//     path_points.push_back(PointFactory::ToPathPoint(
//         ref_point.x() + dx, ref_point.y() + dy, 0.0, s - adc_s,
//         ref_point.heading(), ref_point.kappa(), ref_point.dkappa()));
//   }
//   path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
// }

// bool LaneFollowStage::RetrieveLastFramePathProfile(
//     const ReferenceLineInfo* reference_line_info, const Frame* frame,
//     PathData* path_data) {
//   const auto* ptr_last_frame = injector_->frame_history()->Latest();
//   if (ptr_last_frame == nullptr) {
//     std::cout
//         << "Last frame doesn't succeed, fail to retrieve last frame path data" << std::endl;
//     return false;
//   }

//   const auto& last_frame_discretized_path =
//       ptr_last_frame->current_frame_planned_path();

//   path_data->SetDiscretizedPath(last_frame_discretized_path);
//   const auto adc_frenet_frame_point_ =
//       reference_line_info->reference_line().GetFrenetPoint(
//           frame->PlanningStartPoint().path_point());

//   bool trim_success = path_data->LeftTrimWithRefS(adc_frenet_frame_point_);
//   if (!trim_success) {
//     std::cout << "Fail to trim path_data. adc_frenet_frame_point: "
//            << adc_frenet_frame_point_.ShortDebugString() << std::endl;
//     return false;
//   }
//   std::cout << "Use last frame good path to do speed fallback" << std::endl;
//   return true;
// }

// SLPoint LaneFollowStage::GetStopSL(const ObjectStop& stop_decision,
//                                    const ReferenceLine& reference_line) const {
//   SLPoint sl_point;
//   reference_line.XYToSL(stop_decision.stop_point(), &sl_point);
//   return sl_point;
// }

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo

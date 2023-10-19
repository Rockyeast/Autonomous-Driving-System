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

#pragma once

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
class ReferenceLineInfo {
 public:
  enum class LaneType { LeftForward, LeftReverse, RightForward, RightReverse };
  ReferenceLineInfo() = default;

  ReferenceLineInfo(const common::VehicleState& vehicle_state,
                    const common::TrajectoryPoint& adc_planning_point,
                    const ReferenceLine& reference_line);

  // bool Init(const std::vector<const Obstacle*>& obstacles);

  // bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  // Obstacle* AddObstacle(const Obstacle* obstacle);

  const common::VehicleState& vehicle_state() const { return vehicle_state_; }
  // 获取参考线信息
  const ReferenceLine& reference_line() const;
  ReferenceLine* mutable_reference_line();

  // 设置最终输出轨迹
  void SetTrajectory(const DiscretizedTrajectory& trajectory);
  const DiscretizedTrajectory& trajectory() const;

  // cost和优先级值
  double Cost() const { return cost_; }
  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }
  double PriorityCost() const { return priority_cost_; }
  void SetPriorityCost(double cost) { priority_cost_ = cost; }

  // 巡航速度
  void SetCruiseSpeed(double speed) { cruise_speed_ = speed; }
  double GetCruiseSpeed() const;

  // TODO(wushangzhe):对于变道时查找旁边参考线有用，待编写
  // bool GetNeighborLaneInfo(const ReferenceLineInfo::LaneType lane_type,
  //                          const double s, hdmap::Id* ptr_lane_id,
  //                          double* ptr_lane_width) const;

  /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/
  // bool IsStartFrom(const ReferenceLineInfo& previous_reference_line_info) const;

  planning_internal::Debug* mutable_debug() { return &debug_; }
  const planning_internal::Debug& debug() const { return debug_; }

  // 路径规划 速度规划 对象、指针
  const PathData& path_data() const;
  const PathData& fallback_path_data() const;
  const SpeedData& speed_data() const;
  const SpeedLimit& optimize_speed_limit() const;
  PathData* mutable_path_data();
  PathData* mutable_fallback_path_data();
  SpeedData* mutable_speed_data();
  SpeedLimit* mutable_optimize_speed_limit();
  

  void SetPathData(const PathData &path_data) { path_data_ = std::move(path_data); }
  void SetOptimizeSpeedLimit(const SpeedLimit &optimize_speed_limit) { optimize_speed_limit_ = std::move(optimize_speed_limit); }

  // 将速度规划和路径规划结果合并
  bool CombinePathAndSpeedProfile(
      const double relative_time, const double start_s,
      DiscretizedTrajectory* discretized_trajectory);

  // adjust trajectory if it starts from cur_vehicle postion rather planning
  // init point from upstream
  // bool AdjustTrajectoryWhichStartsFromCurrentPos(
  //     const common::TrajectoryPoint& planning_start_point,
  //     const std::vector<common::TrajectoryPoint>& trajectory,
  //     DiscretizedTrajectory* adjusted_trajectory);

  std::string PathSpeedDebugString() const;

  // 判断是否是变道的参考线，有用，没实现
  bool IsChangeLanePath() const;

  // 没实现
  bool IsNeighborLanePath() const;

  // 设置是否可行驶
  void SetDrivable(bool drivable);
  bool IsDrivable() const;

  // 路权
  ADCTrajectory::RightOfWayStatus GetRightOfWayStatus() const;

  // 偏移其他参考线
  double OffsetToOtherReferenceLine() const {
    return offset_to_other_reference_line_;
  }
  void SetOffsetToOtherReferenceLine(const double offset) {
    offset_to_other_reference_line_ = offset;
  }

  // 候选边界、轨迹
  const std::vector<PathBoundary>& GetCandidatePathBoundaries() const;

  void SetCandidatePathBoundaries(
      std::vector<PathBoundary>&& candidate_path_boundaries);

  const std::vector<PathData>& GetCandidatePathData() const;

  void SetCandidatePathData(std::vector<PathData>&& candidate_path_data);

  bool is_path_lane_borrow() const { return is_path_lane_borrow_; }
  void set_is_path_lane_borrow(const bool is_path_lane_borrow) {
    is_path_lane_borrow_ = is_path_lane_borrow;
  }

  void set_is_on_reference_line() { is_on_reference_line_ = true; }

  // 优先级
  uint32_t GetPriority() const { return reference_line_.GetPriority(); }

  void SetPriority(uint32_t priority) { reference_line_.SetPriority(priority); }

  // 轨迹的类型
  void set_trajectory_type(
      const ADCTrajectory::TrajectoryType trajectory_type) {
    trajectory_type_ = trajectory_type;
  }

  ADCTrajectory::TrajectoryType trajectory_type() const {
    return trajectory_type_;
  }

  // ST图
  StGraphData* mutable_st_graph_data() { return &st_graph_data_; }

  const StGraphData& st_graph_data() { return st_graph_data_; }

  // 路径重用标志
  void set_path_reusable(const bool path_reusable) {
    path_reusable_ = path_reusable;
  }

  bool path_reusable() const { return path_reusable_; }

 private:
  static std::unordered_map<std::string, bool> junction_right_of_way_map_;
  const common::VehicleState vehicle_state_;
  const common::TrajectoryPoint adc_planning_point_;
  ReferenceLine reference_line_; //参考线

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  double cost_ = 0.0;

  bool is_drivable_ = true;  // 是否可以行驶 

  // Obstacle* blocking_obstacle_; // 记录下阻塞的障碍物

  std::vector<PathBoundary> candidate_path_boundaries_;  // 记录路径规划得出的边界信息（可以有多条边界）
  std::vector<PathData> candidate_path_data_;  // 记录候选路径（可用于路径规划DP）

  PathData fallback_path_data_;   // 路径规划失败时，使用的轨迹
  PathData path_data_;  // 最终路径规划得出的轨迹
  SpeedData speed_data_;  // 最终速度规划得出的轨迹

  DiscretizedTrajectory discretized_trajectory_; // 最终路径+速度规划结果轨迹

  planning_internal::Debug debug_;

  bool is_on_reference_line_ = false; // 自车是否在该参考线上

  bool is_path_lane_borrow_ = false; // 是否是借道

  ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED; //路权信息

  double offset_to_other_reference_line_ = 0.0; // 和其他参考线的偏移距离

  double priority_cost_ = 0.0; // 参考线cost值

  ADCTrajectory::TrajectoryType trajectory_type_ = ADCTrajectory::UNKNOWN; // 轨迹类型

  // std::vector<std::pair<OverlapType, hdmap::PathOverlap>>
  //     first_encounter_overlaps_;

  StGraphData st_graph_data_;  // 存放st图数据、限速等信息 (不用这个，后期删掉)

  // 新增变量，从st_graph_data_中拆出来的，要和reference_line中的限速区分开来
  // reference_line中的限速是高精地图给出的限速信息，这个optimize_speed_limit_是用于速度QP，根据地图、曲率、障碍物等计算出来的限速
  SpeedLimit optimize_speed_limit_;  

  common::VehicleSignal vehicle_signal_;

  double cruise_speed_ = 0.0;  // 巡航下的速度

  bool path_reusable_ = false;  // 是否路径重用（当前是要一直重用）
};

}  // namespace planning
}  // namespace apollo

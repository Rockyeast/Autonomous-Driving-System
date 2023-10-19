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

#include <list>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/math/vec2d.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/planning/common/path/discretized_path.h"

namespace apollo {
namespace planning {

/**
 * @class Frame
 *
 * @brief Frame类储存了一个规划周期的所有数据
 */

class Frame {
 public:
   //Frame类的构造函数，参数是序列号
  explicit Frame(uint32_t sequence_num);

//有参构造函数
  Frame(uint32_t sequence_num, 
        const common::VehicleState &vehicle_state,
        std::list<ReferenceLineInfo> *reference_line_info,
        std::vector<Obstacle*> obstacles);

//TODO(wushangzhe)：Frame类带参构造函数，跟上面的构造函数区别就是没有最后一个参数参考线提供器，暂时没用
  Frame(uint32_t sequence_num, 
        const common::VehicleState &vehicle_state,
        std::vector<Obstacle*> obstacles);

  //析构函数
  virtual ~Frame() = default;

  //设置Frame类的数据成员规划起始点planning_start_point_
  void set_planning_start_point(const common::TrajectoryPoint & planning_start_point){
    planning_start_point_ = std::move(planning_start_point);
  }

  //获取Frame类的数据成员规划起始点planning_start_point_
  const common::TrajectoryPoint &PlanningStartPoint() const;

// 返回帧序列号
  uint32_t SequenceNum() const;

// 返回参考线信息类对象
  const std::list<ReferenceLineInfo> &reference_line_info() const;
  
// 返回参考线信息对象指针
  std::list<ReferenceLineInfo> *mutable_reference_line_info();

// 在障碍物列表中寻找输入参数id对应的障碍物对象
  Obstacle *Find(const std::string &id);

// 返回最终使用的驾驶参考线信息
  const ReferenceLineInfo *FindDriveReferenceLineInfo();

// 返回目标道路参考线（变道），目前没用
  const ReferenceLineInfo *FindTargetReferenceLineInfo();

// 查找规划失败的参考线信息，目前没用
  const ReferenceLineInfo *FindFailedReferenceLineInfo();

// 返回Frame类的数据成员驾驶参考线信息类对象
  const ReferenceLineInfo *DriveReferenceLineInfo() const;

// 返回障碍物列表obstacles_
  const std::vector<Obstacle*> obstacles() const;

/**
 * TODO(wushangzhe)：用车道的宽度创建虚拟的静态障碍物，主要是针对虚拟的停止墙
 */
  const Obstacle *CreateStopObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_s);

/**
 * TODO(wushangzhe)：用车道宽度创建虚拟静态障碍物对象，主要是用于虚拟停止墙
 */
  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
                                     const std::string &lane_id,
                                     const double lane_s);

/**
 * TODO(wushangzhe)：构建虚拟静止障碍物
 */
  const Obstacle *CreateStaticObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_start_s,
      const double obstacle_end_s);

// 返回车辆自身状态
  const common::VehicleState &vehicle_state() const;

// debug打印帧序列号
  std::string DebugString() const;
// TODO(wushangzhe)：debug全局信息打印，待完善
  void RecordInputDebug(planning_internal::Debug *debug);

// TODO(wushangzhe)：对齐规划起始时间和障碍物时间戳
  static void AlignPredictionTime(
      const double planning_start_time,
      prediction::PredictionObstacles *prediction_obstacles);

//设置当前frame帧的规划轨迹，用ADCTrajectory类轨迹对象
  void set_current_frame_planned_trajectory(
      ADCTrajectory current_frame_planned_trajectory) {
    current_frame_planned_trajectory_ =
        std::move(current_frame_planned_trajectory);
  }

//返回当前frame的规划轨迹ADCTrajectory类对象
  const ADCTrajectory &current_frame_planned_trajectory() const {
    return current_frame_planned_trajectory_;
  }

//设置当前frame的离散路径 轨迹trajectory=路径path + 速度规划
  void set_current_frame_planned_path(
      DiscretizedPath current_frame_planned_path) {
    current_frame_planned_path_ = std::move(current_frame_planned_path);
  }

//返回当前frame的离散路径对象
  const DiscretizedPath &current_frame_planned_path() const {
    return current_frame_planned_path_;
  }

  const bool is_near_destination() const {
    return is_near_destination_;
  }

//获取Frame类数据成员障碍物列表
  std::vector<Obstacle*> GetObstacleList() {
    return obstacles_;
  }

 private:
  //TODO(wushangzhe)：根据障碍物id和障碍物对应的边界盒对象作为参数去调用构建虚拟障碍物的函数，
  //返回一个虚拟障碍物对象类
  const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
                                              const common::math::Box2d &box);

  //TODO(wushangzhe):增加一个障碍物对象到类数据成员障碍物列表里，计划用于增加虚拟障碍物
  void AddObstacle(Obstacle &obstacle);

 private:
  // 帧序列号 TODO(wushangzhe):会不会超过数据容量界限？
  uint32_t sequence_num_ = 0;
  // 规划起始点，不在上层给出，由规划模块自己计算得出
  common::TrajectoryPoint planning_start_point_;
  // 车辆状态
  common::VehicleState vehicle_state_;
  // 参考线列表
  std::list<ReferenceLineInfo> reference_line_info_;

  // 是否接近目标点 TODO(wushangzhe)：没用
  bool is_near_destination_ = false;

  // 车辆最终选择的道路参考线
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  // 障碍物信息
  std::vector<Obstacle*> obstacles_;

  // 当前帧的规划输出结果，含有路径规划+速度规划合成后的最终结果，成员变量里trajectory_point：最终轨迹点，path_point：不带速度规划的离散路径信息
  ADCTrajectory current_frame_planned_trajectory_;

  // 当前帧的路径点,只带有路径规划的结果（继承的path_point）
  DiscretizedPath current_frame_planned_path_;

};


/**
 * 额外定义历史帧类，用于轨迹拼接，只存上一帧
*/
class FrameHistory : public IndexedQueue<uint32_t, Frame> { // 继承一个由frame组成的队列
 public:
  //frame.h里定义了一个FrameHistory类也定义了一个Frame
  //储存的最大历史Frame的数量，默认为1
  FrameHistory();
};

}  // namespace planning
}  // namespace apollo

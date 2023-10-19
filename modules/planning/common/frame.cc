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
 * @file frame.cc
 **/
#include "modules/planning/common/frame.h"

#include <algorithm>
#include <limits>
#include "absl/strings/str_cat.h"
// #include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
// #include "modules/common/vehicle_state/vehicle_state_provider.h"
// #include "modules/planning/common/feature_output.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
// #include "modules/planning/common/util/util.h"

namespace apollo {
namespace planning {
using apollo::common::ErrorCode; 
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::prediction::PredictionObstacles;

// 历史Frame数据的队列
// 储存的最大历史Frame的数量，默认为1
FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_frame_history_num) {}

// 序列号
Frame::Frame(uint32_t sequence_num)
    : sequence_num_(sequence_num) {}

Frame::Frame(uint32_t sequence_num, 
             const common::VehicleState &vehicle_state,
             std::list<ReferenceLineInfo> *reference_line_info,
             std::vector<Obstacle*> obstacles)
    : sequence_num_(sequence_num),
      vehicle_state_(vehicle_state){
        reference_line_info_ = std::move(*reference_line_info);
        obstacles_ = std::move(obstacles);
      }

Frame::Frame(uint32_t sequence_num, 
             const common::VehicleState &vehicle_state,
             std::vector<Obstacle*> obstacles)
    : Frame(sequence_num, vehicle_state, nullptr, obstacles) {}

// 获取规划起始点planning_start_point_
const common::TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

// 获取车辆状态vehicle_state_
const common::VehicleState &Frame::vehicle_state() const {
  return vehicle_state_;
}

// 获取参考线信息类对象
const std::list<ReferenceLineInfo> &Frame::reference_line_info() const {
  return reference_line_info_;
}

// 获取参考线信息类对象指针
std::list<ReferenceLineInfo> *Frame::mutable_reference_line_info() {
  return &reference_line_info_;
}

/**
 * TODO(wushangzhe):用车道的宽度创建虚拟的静态障碍物，主要是针对虚拟的停止墙
 */
const Obstacle *Frame::CreateStopObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_s) {
  // //如果参考线信息类对象列表为空，报错返回
  // if (reference_line_info == nullptr) {
  //   AERROR << "reference_line_info nullptr";
  //   return nullptr;
  // }

  // //获取输入参数reference_line_info参考线信息里的参考线
  // const auto &reference_line = reference_line_info->reference_line();
  // //障碍物盒对应的中心点的frenet系s坐标=障碍物s坐标 + 0.1 / 2.0
  // //virtual_stop_wall_length是gflags文件中定义的变量，为0.1
  // const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;
  // //用参考线对象根据障碍物盒中心点s坐标获取其对应的参考点对象
  // auto box_center = reference_line.GetReferencePoint(box_center_s);
  // //获取这个障碍物盒中心点对应参考线上的参考点的Heading角
  // double heading = reference_line.GetReferencePoint(obstacle_s).heading();
  // //定义虚拟障碍物盒的宽度为4.0m，宽度对应着车道宽度方向
  // //长度对应着纵向
  // static constexpr double kStopWallWidth = 4.0;
  // //构建虚拟障碍物停止墙边界盒，用障碍物盒的中心点对应的参考点对象，障碍物盒的中心点对
  // //应的参考线上参考点对象heading角，边界盒长度0.1m，边界盒宽度4.0m
  // Box2d stop_wall_box{box_center, heading, FLAGS_virtual_stop_wall_length,
  //                     kStopWallWidth};

  // //然后通过构建的虚拟障碍物的边界盒对象stop_wall_box和障碍物id去创建虚拟障碍物
  // return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * TODO(wushangzhe):用车道宽度创建虚拟静态障碍物对象，主要是用于虚拟停止墙
 */
const Obstacle *Frame::CreateStopObstacle(const std::string &obstacle_id,
                                          const std::string &lane_id,
                                          const double lane_s) {
  // //判断高精地图是否为空？
  // if (!hdmap_) {
  //   AERROR << "Invalid HD Map.";
  //   return nullptr;
  // }
  // //在Frame类的高精度地图对象上根据车道Id去获取车道对象
  // const auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
  // //如果没有找到对应id的车道报错返回
  // if (!lane) {
  //   AERROR << "Failed to find lane[" << lane_id << "]";
  //   return nullptr;
  // }

  // //终点的车道s坐标就是输入参数lane_s
  // double dest_lane_s = std::max(0.0, lane_s);
  // //获取终点的车道s坐标对应的ENU大地系坐标点
  // auto dest_point = lane->GetSmoothPoint(dest_lane_s);

  // //定义两个临时变量，车道中心线左边宽度，车道中心线右边宽度，初始定义为0
  // double lane_left_width = 0.0;
  // double lane_right_width = 0.0;
  // //用参数lane_id对应的车道对象lane去获取终点dest_lane_s处的车道中心线左右宽度。
  // //获取的结果放入引用变量lane_left_width，lane_right_width
  // lane->GetWidth(dest_lane_s, &lane_left_width, &lane_right_width);
  // //构建停止墙边界盒对象stop_wall_box
  // //参数终点处x,y 终点处对应的车道heading,默认的虚拟墙长度0.1m，车道宽度(中心线做宽
  // //度 + 中心线右宽度)
  // Box2d stop_wall_box{{dest_point.x(), dest_point.y()},
  //                     lane->Heading(dest_lane_s),
  //                     FLAGS_virtual_stop_wall_length,
  //                     lane_left_width + lane_right_width};
  // //最后返回根据障碍物id和停止墙边界盒对象构建的虚拟障碍物对象。
  // return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * TODO(wushangzhe):用车道宽度创建虚拟静态障碍物对象
 */
 //这个函数主要是根据3个输入参数：障碍物frenet系下的起始s坐标和终点s坐标，以及障碍物的
 //id，在参考线信息类对象上构建虚拟障碍物
const Obstacle *Frame::CreateStaticObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  // //检查输入参数参考线信息类对象是否为空?
  // if (reference_line_info == nullptr) {
  //   AERROR << "reference_line_info nullptr";
  //   return nullptr;
  // }
  // //获取输入参数reference_line_info上的参考线reference_line 
  // const auto &reference_line = reference_line_info->reference_line();

  // //这一块是将输入参数障碍物起始位置从SL坐标转化成XY坐标obstacle_start_xy
  // common::SLPoint sl_point;
  // sl_point.set_s(obstacle_start_s);
  // sl_point.set_l(0.0);
  // common::math::Vec2d obstacle_start_xy;
  // if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
  //   AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
  //   return nullptr;
  // }

  // //这一块是将输入参数障碍物终点位置从SL坐标转化成XY坐标obstacle_end_s
  // sl_point.set_s(obstacle_end_s);
  // sl_point.set_l(0.0);
  // common::math::Vec2d obstacle_end_xy;
  // if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
  //   AERROR << "Failed to get end_xy from sl: " << sl_point.DebugString();
  //   return nullptr;
  // }

  // double left_lane_width = 0.0;
  // double right_lane_width = 0.0;
  // //获取障碍物起始点处对应的车道中心线左右宽度
  // if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,
  //                                  &right_lane_width)) {
  //   AERROR << "Failed to get lane width at s[" << obstacle_start_s << "]";
  //   return nullptr;
  // }

  // //根据障碍物起始点，终点的xy坐标构成的二维线段类对象
  // //以及车道在虚拟障碍物起始点处的总宽度去构建
  // //构建障碍物的二维障碍物边界盒对象obstacle_box
  // common::math::Box2d obstacle_box{
  //     common::math::LineSegment2d(obstacle_start_xy, obstacle_end_xy),
  //     left_lane_width + right_lane_width};

  // //根据障碍物id和障碍物对应的边界盒对象作为参数去调用构建虚拟障碍物的函数，
  // //返回一个虚拟障碍物对象类
  // return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}

// TODO(wushangzhe):创建虚拟障碍物对象
const Obstacle *Frame::CreateStaticVirtualObstacle(const std::string &id,
                                                   const Box2d &box) {
  // //在障碍物列表中寻找输入参数id对应的障碍物对象
  // const auto *object = obstacles_.Find(id);
  // //如果object不为空，说明找到了对应id的障碍物，障碍物还没创建就已经存在，报警告
  // //并返回
  // if (object) {
  //   AWARN << "obstacle " << id << " already exist.";
  //   return object;
  // }
  // //如果上面没进if也就是说该id还没有在障碍物列表中被创建
  // //用指针ptr指向障碍物列表新增障碍物的地址，这个新增障碍物是以id和二维边界盒为参数调用
  // //Obstacle类的成员函数去创建虚拟障碍物
  // auto *ptr =
  //     obstacles_.Add(id, *Obstacle::CreateStaticVirtualObstacles(id, box));
  // //创建失败的话，报错
  // if (!ptr) {
  //   AERROR << "Failed to create virtual obstacle " << id;
  // }
  // //返回创建的虚拟障碍物的指针
  // return ptr;
}



//返回Frame类数据成员序列号
uint32_t Frame::SequenceNum() const { return sequence_num_; }

// debug用，记录序列号，后续可添加多些信息 todo
std::string Frame::DebugString() const {
  return absl::StrCat("Frame: ", sequence_num_);
}

// TODO:(wushangzhe)：全局debug，待完善
// modules\planning\proto\planning_internal.proto
// 包含规划过程所有相关数据，自车位置，底盘反馈，起始点，路径，速度规划，st图，sl坐标，障碍物，信号灯，前方畅通距离，场景信息等等
void Frame::RecordInputDebug(planning_internal::Debug *debug) {
  // //如果debug为空，直接返回
  // if (!debug) {
  //   ADEBUG << "Skip record input into debug";
  //   return;
  // }
  // //用指针planning_debug_data指向输入参数debug里的planning_data
  // //mutable可以修改被const修饰的变量或成员
  // auto *planning_debug_data = debug->mutable_planning_data();
  // //用指针adc_position指向输入参数debug里的adc_position
  // auto *adc_position = planning_debug_data->mutable_adc_position();
  // //通过指针adc_position设定debug里的planning_debug_data里的自车位置
  // //从local_view_进行拷贝定位信息
  // adc_position->CopyFrom(*local_view_.localization_estimate);

  // //同理设定输入参数debug里的planning_debug_data里的chassis底盘信息
  // //从local_view_进行拷贝底盘信息
  // auto debug_chassis = planning_debug_data->mutable_chassis();
  // debug_chassis->CopyFrom(*local_view_.chassis);

  // //如果不用导航模式的话，设定输入参数debug里的planning_debug_data里的routing全局路
  // //径规划信息，同样是从local_view_拷贝
  // //导航模式就不需要routing了，我理解导航模式就是循迹模式？导航模式通常不用，不重要
  // if (!FLAGS_use_navigation_mode) {
  //   auto debug_routing = planning_debug_data->mutable_routing();
  //   debug_routing->CopyFrom(*local_view_.routing);
  // }

  // //从local_view_里拷贝预测障碍物列表的header到输入参数debug里的planning_debug_data
  // //里的prediction_header
  // planning_debug_data->mutable_prediction_header()->CopyFrom(
  //     local_view_.prediction_obstacles->header());
}

// TODO(wushangzhe):对齐时间
void Frame::AlignPredictionTime(const double planning_start_time,
                                PredictionObstacles *prediction_obstacles) {
  // //如果预测障碍物列表为空 或 预测障碍物列表没有header 或预测障碍物列表的header里
  // //没有时间戳，只要任意一个发生就直接返回
  // if (!prediction_obstacles || !prediction_obstacles->has_header() ||
  //     !prediction_obstacles->header().has_timestamp_sec()) {
  //   return;
  // }
  // //获取输入参数预测障碍物列表的header时间戳prediction_header_time
  // double prediction_header_time =
  //     prediction_obstacles->header().timestamp_sec();
  // //遍历预测障碍物列表
  // for (auto &obstacle : *prediction_obstacles->mutable_prediction_obstacle()) {
  //   //遍历障碍物轨迹
  //   for (auto &trajectory : *obstacle.mutable_trajectory()) {
  //     //遍历轨迹点
  //     for (auto &point : *trajectory.mutable_trajectory_point()) {
  //       //修改预测障碍物轨迹点的相对时间为 预测头部时间 + 预测轨迹点相对时间 - 规划起
  //       //始时间，其实就是为了将预测障碍物列表的header时间与planning的起始时间对齐
  //       point.set_relative_time(prediction_header_time + point.relative_time() -
  //                               planning_start_time);
  //     }
  //     //如果障碍物的预测轨迹点非空 且 起始轨迹点的相对时间<0
  //     if (!trajectory.trajectory_point().empty() &&
  //         trajectory.trajectory_point().begin()->relative_time() < 0) {
  //       auto it = trajectory.trajectory_point().begin();
  //       //如果预测障碍物列表相对header时间小于0，则遍历找到预测障碍物轨迹上相对时间不
  //       //小于0的轨迹点
  //       while (it != trajectory.trajectory_point().end() &&
  //              it->relative_time() < 0) {
  //         ++it;
  //       }
  //       //清除掉这个预测障碍物相对时间<0的部分
  //       trajectory.mutable_trajectory_point()->erase(
  //           trajectory.trajectory_point().begin(), it);
  //     }
  //   }
  // }
}

// 由障碍物id查找障碍物对象
Obstacle *Frame::Find(const std::string &id) { 
  for (Obstacle* obstacle : obstacles_) {
    if(obstacle->Id() == id){
      return obstacle;
    }
  }
}

//增加一个障碍物对象到类数据成员障碍物列表里(不确定是否有bug)
void Frame::AddObstacle(Obstacle &obstacle) {
  // obstacles_.Add(obstacle.Id(), obstacle);
  obstacles_.push_back(&obstacle);
}

// 找最终使用的驾驶参考线信息
const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  //初始化最小的cost为无穷大，就是double能表示的最大数
  double min_cost = std::numeric_limits<double>::infinity();
  //初始定义Frame类的数据成员驾驶参考线drive_reference_line_info_信息对象为空指针
  drive_reference_line_info_ = nullptr;
  //遍历Frame类里的数据成员的道路参考线列表reference_line_info_
  //其实这个函数就是找到Frame类数据成员reference_line_info_道路参考线列表中cost最小且可行驶的道路参考线
  for (const auto &reference_line_info : reference_line_info_) {
    //如果遍历的这条道路参考线可驾驶，且cost小于最小cost（初始定义为无穷大）
    //那么就更新最小的cost，同时更新驾驶道路参考线对象drive_reference_line_info_
    //更新为这条cost更小的道路参考线
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      drive_reference_line_info_ = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  return drive_reference_line_info_;
}

//找目标道路参考线，没用
const ReferenceLineInfo *Frame::FindTargetReferenceLineInfo() {
  // //初始化定义输出结果的道路参考线对象为空
  // const ReferenceLineInfo *target_reference_line_info = nullptr;
  // //遍历Frame类的数据成员reference_line_info_道路参考线列表
  // for (const auto &reference_line_info : reference_line_info_) {
  //   //如果找到变道路径参考线就直接返回，所以这个函数其实是找变道参考线？
  //   if (reference_line_info.IsChangeLanePath()) {
  //     return &reference_line_info;
  //   }
  //   target_reference_line_info = &reference_line_info;
  // }
  // return target_reference_line_info;
}

//查找失败的参考线信息，返回道路参考线信息类ReferenceLineInfo类对象
//遍历道路参考线列表，找到不可驾驶的变道参考线信息类对象?
const ReferenceLineInfo *Frame::FindFailedReferenceLineInfo() {
  // //遍历道路参考线列表，找到不可驾驶的变道参考线信息类对象?
  // for (const auto &reference_line_info : reference_line_info_) {
  //   // Find the unsuccessful lane-change path
  //   if (!reference_line_info.IsDrivable() &&
  //       reference_line_info.IsChangeLanePath()) {
  //     return &reference_line_info;
  //   }
  // }
  // return nullptr;
}

//返回Frame类的数据成员驾驶参考线信息类对象
const ReferenceLineInfo *Frame::DriveReferenceLineInfo() const {
  return drive_reference_line_info_;
}

//返回Frame类的数据成员障碍物列表obstacles_
const std::vector<Obstacle *> Frame::obstacles() const {
  return obstacles_;
}

}  // namespace planning
}  // namespace apollo
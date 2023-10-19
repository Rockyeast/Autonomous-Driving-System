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

#include "modules/planning/tasks/deciders/st_generate_decider/st_generate_decider.h"

#include <limits>
#include <memory>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/common/math/vec2d.h"
#include <algorithm>


namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::VehicleParam;
using apollo::common::SLPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

StGenerateDecider::StGenerateDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  // TODO(wushangzhe):这个检查不痛不痒
  if(!config_.has_st_generate_decider_config()){
    std::cout << "load StGenerateDecider config error !!!" << std::endl;
  }
  // ACHECK(config_.has_lane_change_decider_config());
}

// 每个task处理入口
bool StGenerateDecider::Process(FrameHistory* frame_history, 
    Frame* frame, ReferenceLineInfo* const current_reference_line_info) {

  // 按以下步骤进行task编写

  // 1.合理性检查：检查传入参数是否为空

  // 2.获取配置文件参数、frame中所需数据（同时进行合理性检查）
  
  // 3.逻辑代码编写。（建议大块逻辑代码放到Process之外处理，或者另开文件类进行处理，保持Process简洁易读）

  // 4.运行成功：结果保存至frame对应位置，return true
  //   运行失败: 相应处理措施，输出失败原因，return false

  // 检查当前车道线是否为空
  if(current_reference_line_info->reference_line().reference_points().empty()){
    std::cout << "Reference line empty !" << std::endl;
    return false;
  }
  // 检查路径规划结果
  const PathData path_data = current_reference_line_info->path_data();
  if(path_data.discretized_path().size() < 2){
    std::cout << "Path data point too few !" << std::endl;
    return false;
  }
  // 检查规划起始点
  if(!frame->PlanningStartPoint().has_path_point()){
    std::cout << "Planning Start Point is empty !" << std::endl;
    return false;
  }

  bool ret;

  // 获取配置文件全局参数
  const auto& st_generate_decider_config = config_.st_generate_decider_config();  // modules/planning/conf/planning_config.pb.txt
  // std::cout << "test1 = " << st_generate_decider_config.test1() << std::endl;

  struct timeval start_timestamp, end_timestamp;  // 计算耗时
  gettimeofday(&start_timestamp, NULL);

  // 计算st图
  ret = CreatSTGraphByParallelogram(frame, path_data, st_generate_decider_config);
  if(!ret){
    std::cout << "st generate fail !" << std::endl;
    return false;
  }

  gettimeofday(&end_timestamp, NULL);
  long time_diff_us = (end_timestamp.tv_sec - start_timestamp.tv_sec) * 1000000 
                + (end_timestamp.tv_usec - start_timestamp.tv_usec);
  std::cout << "st generate part get time spend: " << time_diff_us << " us." << std::endl;
  start_timestamp = end_timestamp;

  // 计算QP限速信息
  ret = CalculateSpeedLimit(st_generate_decider_config, frame->obstacles(), current_reference_line_info, path_data);

  gettimeofday(&end_timestamp, NULL);
  time_diff_us = (end_timestamp.tv_sec - start_timestamp.tv_sec) * 1000000 
                + (end_timestamp.tv_usec - start_timestamp.tv_usec);
  std::cout << "Calculate SpeedLimit part get time spend: " << time_diff_us << " us." << std::endl;


  if(!ret){
    // 规划失败下的处理：
    // .......
    // .......
    return false;
  }
  // 规划成功的处理：赋值结果为下一个task准备
  else{
    // .......
    // .......
    return true;
  }
}

/**
 * 生成平行四边形的ST图
 * 存放在每个障碍物类obstacle的成员属性path_st_boundary_（在一帧中若要多次规划记得清空
 * 类型为STBoundary，由类内的：std::vector<STPoint> upper_points_; std::vector<STPoint> lower_points_;存储
 * 如下图示意：
 * S|                       upper_points_[1]        * upper_points_[2]       ...      
 *  | upper_points_[0]          *
 *  |  *  
 *  |                       lower_points_[1]        * upper_points_[2]       ...
 *  | lower_points_[0]           *   
 *  |__*_______________________________________________________________
 *                                                                   T
*/
bool StGenerateDecider::CreatSTGraphByParallelogram(Frame* frame, PathData path_data, StGenerateDeciderConfig config){

  // TODO(wushangzhe):安全距离，随便设置的，需要写入config
  double buffer = 0.5;

  // 获取车辆参数
  const VehicleParam& vehicle_param = common::VehicleConfigHelper::GetConfig().vehicle_param();

  // 遍历所有障碍物
  for(auto obstacle: frame->obstacles()){

    // 上下边界
    std::vector<STPoint> upper_points;
    std::vector<STPoint> lower_points;

    // 跳过静态和虚拟障碍物
    if(obstacle->IsStatic() || obstacle->IsVirtual()){
      continue;
    }

    // 遍历动态障碍物的预测轨迹
    const auto& trajectory = obstacle->Trajectory();
    // 对于没有预测轨迹的，当成静态障碍物处理
    if(trajectory.trajectory_point().empty()){
      std::cout << "dynamics obstacle no trajectory !" << std::endl;
      // 遍历离散路径点
      for (const auto& curr_point_on_path : path_data.discretized_path()) {
        // 获取障碍物的boundingbox
        perception::PerceptionObstacle perception_obstacle = obstacle->Perception();
        const Box2d& obs_box = common::math::Box2d({perception_obstacle.position().x(), perception_obstacle.position().y()},
                                perception_obstacle.theta(),
                                perception_obstacle.length(),
                                perception_obstacle.width());
        Box2d adc_box = GetADCBoundingBox(curr_point_on_path, vehicle_param, buffer);
        if (CheckCollision(adc_box, obs_box)) {
          // 有碰撞可能才投影到ST图上，无碰撞风险就不管了
          const double backward_distance = -vehicle_param.front_edge_to_center();
          const double forward_distance = obs_box.length();
          double low_s =
              std::fmax(0.0, curr_point_on_path.s() + backward_distance);
          double high_s = std::fmin(path_data.discretized_path().Length(),
                                    curr_point_on_path.s() + forward_distance);  // 上边界的选取待测试
          // 静止的障碍物在ST图中就是一个矩形
          lower_points.emplace_back(low_s, 0.0);
          lower_points.emplace_back(low_s, config.total_time());
          upper_points.emplace_back(high_s, 0.0);
          upper_points.emplace_back(high_s, config.total_time());
          break;
        }
      }
    }
    // 动态障碍物有轨迹
    else{
      // TODO(wushangzhe)：针对算力优化，对路径规划path进行稀疏采样，减少计算量
      // const int default_num_point = 50;  // 默认只取50个点
      // DiscretizedPath discretized_path;
      // if (path_points.size() > 2 * default_num_point) { // 多于100个点才稀疏采样，否则直接取全部规划路径点
      //   const auto ratio = path_points.size() / default_num_point;
      //   std::vector<PathPoint> sampled_path_points;
      //   for (size_t i = 0; i < path_points.size(); ++i) {
      //     if (i % ratio == 0) {
      //       sampled_path_points.push_back(path_points[i]);
      //     }
      //   }
      //   discretized_path = DiscretizedPath(std::move(sampled_path_points));
      // } else {
      //   discretized_path = DiscretizedPath(path_points);
      // }

      // 遍历障碍物上每一个轨迹点
      for(const auto& trajectory_point: trajectory.trajectory_point()){
        // 获取轨迹点相对轨迹起始点的时间，作为ST图的T
        double relative_time = trajectory_point.relative_time();
        // 得到障碍物在该轨迹点处的boundingbox，根据x，y，theta，length、width
        const Box2d obs_box = obstacle->GetBoundingBox(trajectory_point);
        // 遍历路径规划path上每一个点
        for(int i = 0; i < path_data.discretized_path().size(); i++){
          // 获取自车在该路径规划点的boundingbox，坐标需要从后轴中心转换到自车中心
          Box2d adc_box = GetADCBoundingBox(path_data.discretized_path().at(i), vehicle_param, buffer);
          // 检查两个box的碰撞情况
          if(CheckCollision(adc_box, obs_box)){
            // 有碰撞可能，先确定一个粗略上下s界，然后通过微调，找到最终合适贴合的上下界s
            
            // 当前s
            double path_s = path_data.discretized_path().at(i).s();
            // 和前一个点的距离s
            double delta_s = i? path_data.discretized_path().at(i).s() - path_data.discretized_path().at(i-1).s()
                              : -vehicle_param.front_edge_to_center(); // 不能第一个点就碰撞吧？

            // TODO(wushangzhe):bug，如果这个粗略上界还是有碰撞呢？以及距离多少为合适？
            // 设置回退的距离，设置了两种，可以都试试效果（这个距离肯定是够了的，因为后方的肯定没有碰撞，内循环找到一个交互点就break）
            // const double backward_distance = -vehicle_param.front_edge_to_center(); // 3.075,后轴中心到前保杆距离
            const double backward_distance = -delta_s; // 如果用上一个路径点是不是太近了？
            // 设置前进距离，这里设置的比较大（因为前方未知）
            const double forward_distance = vehicle_param.length() +
                                            vehicle_param.width() +
                                            obs_box.length() + obs_box.width();
            const double default_min_step = 0.1;  // in meters
            const double fine_tuning_step_length = std::fmin(  // 微调时的步长
                default_min_step, delta_s);

            bool find_low = false;
            bool find_high = false;
            double low_s = std::fmax(0.0, path_s + backward_distance);
            double high_s = std::fmin(path_data.discretized_path().Length(), path_s + forward_distance);

            // 按分辨率双向缩小，直到最后获得贴合的上下限
            while (low_s < high_s) {
              if (find_low && find_high) {
                break;
              }
              if (!find_low) {
                const auto& point_low = path_data.discretized_path().Evaluate(
                    low_s + path_data.discretized_path().front().s());
                adc_box = GetADCBoundingBox(point_low, vehicle_param, buffer);
                if (!CheckCollision(adc_box, obs_box)) {
                  low_s += fine_tuning_step_length; // 没有碰撞则继续往上加s
                } else {
                  find_low = true;  // 有碰撞则说明找到了s下边界
                }
              }
              if (!find_high) {
                const auto& point_high = path_data.discretized_path().Evaluate(
                    high_s + path_data.discretized_path().front().s());
                adc_box = GetADCBoundingBox(point_high, vehicle_param, buffer);
                if (!CheckCollision(adc_box, obs_box)) {
                  high_s -= fine_tuning_step_length;
                } else {
                  find_high = true;
                }
              }
            }
            if (find_high && find_low) {
              lower_points.emplace_back(low_s, relative_time);
              upper_points.emplace_back(high_s, relative_time);
            }
            break;
          }
        }
      }

    // back
    // // 2. Go through every point of the predicted obstacle trajectory.
    // // 遍历障碍物预测轨迹点
    // // 外层循环遍历障碍物的每一个轨迹点，计算每个轨迹点的boundingbox
    // // 内层循环判断规划出的路径（被稀疏化）每一个点，和轨迹点的boundingbox做碰撞检测
    // DiscretizedPath discretized_path = DiscretizedPath(path_data.discretized_path());
    // for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
    //   const auto& trajectory_point = trajectory.trajectory_point(i);
    //   // 得到障碍物在该轨迹点处的boundingbox
    //   const Box2d obs_box = obstacle->GetBoundingBox(trajectory_point);

    //   // 得到障碍物在轨迹点处的相对时间
    //   double trajectory_point_time = trajectory_point.relative_time();
    //   static constexpr double kNegtiveTimeThreshold = -1.0;
    //   // 跳过小于阈值时间的轨迹点，但这个阈值设置负数是为了防止什么？
    //   if (trajectory_point_time < kNegtiveTimeThreshold) {
    //     continue;
    //   }

    //   // 步长
    //   const double step_length = vehicle_param.front_edge_to_center(); // 3.705
    //   auto path_len =
    //       std::min(FLAGS_max_trajectory_len, discretized_path.Length());  // 1000m
    //   // Go through every point of the ADC's path. 遍历路径规划（在轨迹上但可能不是离散轨迹点，中间要插值）
    //   for (double path_s = 0.0; path_s < path_len; path_s += step_length) {
    //     // 线性插值计算当前车辆的位置 返回PathPoint类型
    //     const auto curr_adc_path_point =
    //         discretized_path.Evaluate(path_s + discretized_path.front().s());
    //     if (CheckOverlap(curr_adc_path_point, obs_box, buffer)) {
    //       // 有碰撞可能，则先确定一个粗略上下s界，然后通过微调，找到最终合适贴合的上下界s
    //       // Found overlap, start searching with higher resolution
    //       const double backward_distance = -step_length;
    //       const double forward_distance = vehicle_param.length() +
    //                                       vehicle_param.width() +
    //                                       obs_box.length() + obs_box.width();
    //       const double default_min_step = 0.1;  // in meters
    //       const double fine_tuning_step_length = std::fmin(  // 微调时的步长
    //           default_min_step, discretized_path.Length() / 50.0);

    //       bool find_low = false;
    //       bool find_high = false;
    //       double low_s = std::fmax(0.0, path_s + backward_distance);
    //       double high_s =
    //           std::fmin(discretized_path.Length(), path_s + forward_distance);

    //       // Keep shrinking by the resolution bidirectionally until finally
    //       // locating the tight upper and lower bounds.
    //       // 按分辨率双向缩小，直到最后定位紧的上限和下限
    //       while (low_s < high_s) {
    //         if (find_low && find_high) {
    //           break;
    //         }
    //         if (!find_low) {
    //           const auto& point_low = discretized_path.Evaluate(
    //               low_s + discretized_path.front().s());
    //           if (!CheckOverlap(point_low, obs_box, buffer)) {
    //             low_s += fine_tuning_step_length; // 没有碰撞则继续往上加s
    //           } else {
    //             find_low = true;  // 有碰撞则说明找到了s下边界
    //           }
    //         }
    //         if (!find_high) {
    //           const auto& point_high = discretized_path.Evaluate(
    //               high_s + discretized_path.front().s());
    //           if (!CheckOverlap(point_high, obs_box, buffer)) {
    //             high_s -= fine_tuning_step_length;
    //           } else {
    //             find_high = true;
    //           }
    //         }
    //       }
    //       if (find_high && find_low) {
    //         lower_points.emplace_back(
    //             low_s, // 定义的0
    //             trajectory_point_time);
    //         upper_points.emplace_back(
    //             high_s,
    //             trajectory_point_time);
    //       }
    //       break;
    //     }
    //   }
    // }

    }
    // 将st图存入相应障碍物中
    auto boundary = STBoundary::CreateInstance(lower_points, upper_points); //boundary实例化中只保存lower和upper首末两点
    boundary.set_id(obstacle->Id());
    obstacle->set_path_st_boundary(boundary);
  }

  // DebugInfo_st(frame);
  // DebugInfo_trj(frame, path_data);
  std::cout << "prosess done !" << std::endl;
  return true;

}


bool StGenerateDecider::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  // 将参考点从后轴中心转换为ADC中心（质心）
  const VehicleParam& vehicle_param = common::VehicleConfigHelper::GetConfig().vehicle_param();

  Vec2d ego_center_map_frame((vehicle_param.front_edge_to_center() -
                              vehicle_param.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param.left_edge_to_center() -
                              vehicle_param.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param.length(), vehicle_param.width() + l_buffer * 2);

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  return obs_box.HasOverlap(adc_box);
}


/**
 * TODO(wushangzhe)
 * 生成线段形式的ST图
*/
bool StGenerateDecider::CreatSTGraphByLine(Frame* frame, PathData path_data){

}

Box2d StGenerateDecider::GetADCBoundingBox(const PathPoint& path_point, 
                      const VehicleParam& vehicle_param, double& buffer){
  
  // 将路径点从后轴中心转换为ADC中心（质心）
  Vec2d ego_center_map_frame((vehicle_param.front_edge_to_center() -
                              vehicle_param.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param.left_edge_to_center() -
                              vehicle_param.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

  Box2d adc_box(ego_center_map_frame, path_point.theta(),
                vehicle_param.length(), vehicle_param.width() + buffer * 2);
  return adc_box;
}

bool StGenerateDecider::CheckCollision(Box2d& adc_box, const Box2d& obs_box){
  // 分离轴定理计算重叠
  return obs_box.HasOverlap(adc_box);
}

// 计算QP限速
bool StGenerateDecider::CalculateSpeedLimit(StGenerateDeciderConfig config, const std::vector<Obstacle *> obstacles, 
                        ReferenceLineInfo* current_reference_line_info, PathData path_data){

  const auto& discretized_path = path_data.discretized_path();
  const auto& frenet_path = path_data.frenet_frame_path();
  const auto& reference_line = current_reference_line_info->reference_line();
  SpeedLimit speed_limit;
  // 遍历离散路径点
  for (uint32_t i = 0; i < discretized_path.size(); ++i) {
    const double path_s = discretized_path.at(i).s();
    const double reference_line_s = frenet_path.at(i).s();

    // 从高精地图获取限速，目前无，默认27.7
    double speed_limit_from_reference_line =
        reference_line.GetSpeedLimitFromS(reference_line_s);

    // 根据曲率计算限速
    const double speed_limit_from_curvature =
        std::sqrt(config.max_curvature_acceleration_limit() /  //2.0
                  std::fmax(std::fabs(discretized_path.at(i).kappa()),
                            config.minimal_kappa()));  //防止为0情况 0.003

    // TODO(wushangzhe)：根据障碍物计算限速，大车？小车？慢车？
    double speed_limit_from_nearby_obstacles =
        std::numeric_limits<double>::max();
    // 遍历障碍物
    for (const auto* ptr_obstacle : obstacles) {}

    double curr_speed_limit = 0.0;
    curr_speed_limit =
        std::fmax(config.lowest_speed(),
                  std::min({speed_limit_from_reference_line,
                            speed_limit_from_curvature}));
    speed_limit.AppendSpeedLimit(path_s, curr_speed_limit);
  }
  
  // 存入当前参考线中
  current_reference_line_info->SetOptimizeSpeedLimit(speed_limit);
  return true;
}



// debug用，输出ST图信息
void StGenerateDecider::DebugInfo_st(Frame* frame){
  const char* log_file_name = "/st_data.csv";
  DebugInterface* debug_interface = new DebugInterface();
  std::fstream emplanner_log_file = debug_interface->WriteHeaders(log_file_name);
  
  for(auto obs: frame->obstacles()){
    auto boundary = obs->path_st_boundary();
    auto upper_points = boundary.upper_points();
    auto lower_points = boundary.lower_points();
    // x-axis: t; y-axis: s.
    emplanner_log_file << "lt,ls,ut,us,x,y" << std::endl;
    for(int i = 0; i < upper_points.size(); i++){
      emplanner_log_file << lower_points.at(i).t() << ",";
      emplanner_log_file << lower_points.at(i).s() << ",";
      emplanner_log_file << upper_points.at(i).t() << ",";
      emplanner_log_file << upper_points.at(i).s() << ",";
      emplanner_log_file << std::endl;
    }
  }
  delete(debug_interface);
}

// debug用，输出轨迹信息
void StGenerateDecider::DebugInfo_trj(Frame* frame, PathData path_data){
  const char* log_file_name = "/obs_trj.csv";
  DebugInterface* debug_interface = new DebugInterface();
  std::fstream emplanner_log_file = debug_interface->WriteHeaders(log_file_name);
  
  // 输出障碍物预测轨迹
  for(auto obs: frame->obstacles()){
    emplanner_log_file << "x,y" << std::endl;
    for(const auto& trajectory_point: obs->Trajectory().trajectory_point()){
      emplanner_log_file << trajectory_point.path_point().x() << ",";
      emplanner_log_file << trajectory_point.path_point().y() << ",";
      emplanner_log_file << std::endl;
    }
  }
  delete(debug_interface);

  const char* log_file_name2 = "/adc_path.csv";
  DebugInterface* debug_interface2 = new DebugInterface();
  std::fstream emplanner_log_file2 = debug_interface2->WriteHeaders(log_file_name2);
  emplanner_log_file2 << "x,y" << std::endl;
  // 输出障碍物预测轨迹
  for(int i = 0; i < path_data.discretized_path().size(); i++){
      emplanner_log_file2 << path_data.discretized_path().at(i).x() << ",";
      emplanner_log_file2 << path_data.discretized_path().at(i).y() << ",";
      if(i == path_data.discretized_path().size()-1)
        break;
      emplanner_log_file2 << std::endl;
  }
  delete(debug_interface2);
}

}  // namespace planning
}  // namespace apollo

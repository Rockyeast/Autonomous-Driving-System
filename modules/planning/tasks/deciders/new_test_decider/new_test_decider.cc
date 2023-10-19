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

#include "modules/planning/tasks/deciders/new_test_decider/new_test_decider.h"

#include <limits>
#include <memory>

#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::TrajectoryPoint;

NewTestDecider::NewTestDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  // TODO(wushangzhe):缺少对配置文件打开安全检查
  // ACHECK(config_.has_lane_change_decider_config());
}

// 每个task处理入口
bool NewTestDecider::Process(FrameHistory* frame_history, 
    Frame* frame, ReferenceLineInfo* const current_reference_line_info) {

  // 按以下步骤进行task编写

  // 1.合理性检查：检查传入参数是否为空

  // 2.获取配置文件参数、frame中所需数据（同时进行合理性检查）
  
  // 3.逻辑代码编写。（建议大块逻辑代码放到Process之外处理，或者另开文件类进行处理，保持Process简洁易读）

  // 4.运行成功：结果保存至frame对应位置，return true
  //   运行失败: 相应处理措施，输出失败原因，return false

  // 样例：
  // 检查车道线是否为空
  std::list<ReferenceLineInfo>* reference_line_info =
      frame->mutable_reference_line_info();
  if (reference_line_info->empty()) {
    const std::string msg = "Reference lines empty.";
    std::cout << msg;
    return false;
  }

  // 获取配置文件全局参数
  const auto& new_test_decider_config = config_.new_test_decider_config();  // modules/planning/conf/planning_config.pb.txt
  std::cout << "argn1 = " << new_test_decider_config.argn1() << std::endl;

  bool ret = testFunction(frame, current_reference_line_info);
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
 * task 主逻辑代码编写
*/
bool NewTestDecider::testFunction(Frame* frame, ReferenceLineInfo* const current_reference_line_info){

  // 速度规划测试用，赋值规划起始点和路径规划
  
  // 规划起始点
  TrajectoryPoint planning_start_point;
  planning_start_point.set_v(10.0);
  planning_start_point.set_a(0.0);
  planning_start_point.set_relative_time(0.0);
  planning_start_point.set_da(0.0);
  planning_start_point.set_steer(0.0);  //应该是航向角的意思
  PathPoint* start_point = new PathPoint();
  start_point->set_x(0);
  start_point->set_y(0);
  start_point->set_z(0);
  start_point->set_theta(0);
  start_point->set_kappa(0);
  start_point->set_dkappa(0);
  start_point->set_ddkappa(0);
  start_point->set_s(0);
  start_point->set_lane_id("1");
  planning_start_point.set_allocated_path_point(start_point);
  // 赋值起始点
  frame->set_planning_start_point(planning_start_point);

  // 路径规划结果
  PathData path_data;
  // 需要先有参考线才能进行路径赋值等操作
  path_data.SetReferenceLine(current_reference_line_info->mutable_reference_line()); 
  // 构造离散路径点
  std::vector<common::PathPoint> path_points;
  for(int i = 0;i < 200;i++)
  {
    common::PathPoint* path_point = new PathPoint();
    path_point->set_x(i/2.0);
    path_point->set_y(0.0);
    path_point->set_z(0.0);
    path_point->set_theta(0.0);
    path_point->set_kappa(0.0);
    path_point->set_s(i/2.0);
    path_point->set_dkappa(0.0);
    path_point->set_ddkappa(0.0);
    path_points.push_back(*path_point);
  }
  // 构造离散路径轨迹
  DiscretizedPath* discretized_path = new DiscretizedPath(path_points);
  path_data.SetDiscretizedPath(*discretized_path);  //设置离散xy路径的时候，同步已经转换了frenet路径
  // 路径规划结果存入参考线中
  current_reference_line_info->SetPathData(path_data);


  std::cout << "prosess done !" << std::endl;
  return true;

}


}  // namespace planning
}  // namespace apollo

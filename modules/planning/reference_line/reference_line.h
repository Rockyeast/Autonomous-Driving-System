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
 * @file reference_line.h
 **/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/proto/map.pb.h"
#include "modules/map/proto/map_geometry.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/planning/reference_line/reference_point.h"
#include "modules/routing/proto/routing.pb.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "modules/common/math/path_matcher.h"

namespace apollo {
namespace planning {
using apollo::common::PathPoint;

class ReferenceLine {
 public:
  ReferenceLine() = default;
  explicit ReferenceLine(const ReferenceLine& reference_line) = default;
  template <typename Iterator>
  ReferenceLine(const Iterator begin, const Iterator end)
      : reference_points_(begin, end) {}
  explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);  // 测试阶段初始化调用这个构造函数

  // 参考线拼接（由上游模块做），未实现
  bool Stitch(const ReferenceLine& other);

  // 获取s点后distance_backward，前distance_forward之间的Segment道路段（由高精地图得出），未实现
  bool Segment(const double s, const double distance_backward,
               const double distance_forward);

  double Length() const { return path_points_.back().s();}

  // 获取参考线上离散点
  const std::vector<ReferencePoint>& reference_points() const;

  // TODO(wushangzhe):用离散路径点代替原来的参考线点，是有误差的！待改进（应该取参考线平滑点），未测试！
  ReferencePoint GetReferencePoint(const double s) const;

  // TODO(wushangzhe)：把path_point投影到frenet坐标点，待实现
  common::FrenetFramePoint GetFrenetPoint(
      const common::PathPoint& path_point) const;

  // 把轨迹点转换到frenet，输出：[(s,s',s''),(d,d',d'')]
  std::pair<std::array<double, 3>, std::array<double, 3>> ToFrenetFrame(
      const common::TrajectoryPoint& traj_point) const;

  // TODO(wushangzhe)：获取参考线上[start_s,end_s]区间内所有的参考线离散点，未实现
  std::vector<ReferencePoint> GetReferencePoints(double start_s,
                                                 double end_s) const;

  // TODO(wushangzhe)：新增函数，用于根据s找离散路径点上的匹配点，未测试！
  size_t GetIndexFromS(const double s) const;

  // // 把x，y坐标投影到参考线的frenet坐标系上
  // PathPoint GetReferencePoint(const double x, const double y) const;

  // 根据box的四个顶点，得到SL图的边界（start_s, end_s, start_l, end_l）
  // bool GetSLBoundary(const common::math::Box2d& box,
  //                    SLBoundary* const sl_boundary) const;
  // bool GetSLBoundary(const hdmap::Polygon& polygon,
  //                    SLBoundary* const sl_boundary) const;

  bool SLToXY(const common::SLPoint& sl_point,
              common::math::Vec2d* const xy_point) const;

  // TODO(wushangzhe)：把xy坐标投影到参考线frenet坐标系上，未进行测试！
  bool XYToSL(const common::math::Vec2d& xy_point,
              common::SLPoint* const sl_point) const;

  // TODO(wushangzhe):根据s获取当前离左右车道线的距离offest，也得由高精地图给出，测试阶段可以直接给固定车道宽值
  bool GetLaneWidth(const double s, double* const lane_left_width,
                    double* const lane_right_width) const;

  double GetDrivingWidth(const SLBoundary& sl_boundary) const;


  // TODO(wushangzhe)：一般来说道路中心线和参考线不一定是重合的，有一定偏移量，这个值要从高精地图得出，当前不考虑
  bool GetOffsetToMap(const double s, double* l_offset) const;

  // TODO(wushangzhe):根据s获取当前离左右路沿的距离offest，也得由高精地图给出，测试阶段可以直接给车道+路沿宽
  bool GetRoadWidth(const double s, double* const road_left_width,
                    double* const road_right_width) const;

  // TODO(wushangzhe):获取道路类型，由高精地图给出
  // hdmap::Road::Type GetRoadType(const double s) const;

  // 由sl坐标判断是否在该参考线所在的车道内（根据路宽判断）
  bool IsOnLane(const common::SLPoint& sl_point) const;

  // 由sl坐标判断是否在道路内（根据路宽判断）
  bool IsOnRoad(const common::SLPoint& sl_point) const;

  // TODO(wushangzhe):判断该条车道是否被阻塞，待编写
  bool IsBlockRoad(const common::math::Box2d& box2d, double gap) const;

  // 判断传入的box和该车道是否有重叠部分，未实现
  bool HasOverlap(const common::math::Box2d& box) const;

  std::string DebugString() const;

  // 根据s查询道路限速，需要由高精地图给出，测试阶段直接给固定值
  double GetSpeedLimitFromS(const double s) const;

  // 根据自己的决策添加的限速信息
  void AddSpeedLimit(double start_s, double end_s, double speed_limit);

  // 获取参考线的优先级
  uint32_t GetPriority() const { return priority_; }

  // 设置参考线的优先级
  void SetPriority(uint32_t priority) { priority_ = priority; }

  void ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points);

  // const hdmap::Path& GetMapPath() const { return map_path_; }

 private:
  // TODO(wushangzhe):获取线性插值，待实现
  static ReferencePoint Interpolate(const ReferencePoint& p0, const double s0,
                                    const ReferencePoint& p1, const double s1,
                                    const double s);

  // TODO(wushangzhe):获取距离（x,y）点最近的参考线离散点，待实现
  static double FindMinDistancePoint(const ReferencePoint& p0, const double s0,
                                     const ReferencePoint& p1, const double s1,
                                     const double x, const double y);

 private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  std::vector<PathPoint> path_points_;  // 新增变量，用于存放离散后的参考线信息(为了解决frenet转换类型不匹配问题)
  uint32_t priority_ = 0;
};

}  // namespace planning
}  // namespace apollo

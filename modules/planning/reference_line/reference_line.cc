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

#include "reference_line.h"

#include <algorithm>
#include <limits>
#include <unordered_set>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "boost/math/tools/minima.hpp"

#include "modules/common/math/angle.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"


namespace apollo {
namespace planning {

using apollo::common::SLPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::math::Vec2d;
using apollo::common::PathPoint;
using apollo::common::math::PathMatcher;

ReferenceLine::ReferenceLine(
    const std::vector<ReferencePoint>& reference_points)
    : reference_points_(reference_points) {
      ToDiscretizedReferenceLine(reference_points_);
    }

const std::vector<ReferencePoint>& ReferenceLine::reference_points() const {
  return reference_points_;
}

bool ReferenceLine::GetLaneWidth(const double s, double* const lane_left_width,
                                 double* const lane_right_width) const {
  *lane_left_width = 1.75;
  *lane_right_width = 1.75;
}

double ReferenceLine::GetDrivingWidth(const SLBoundary& sl_boundary) const {
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  GetLaneWidth(sl_boundary.start_s(), &lane_left_width, &lane_right_width);

  double driving_width = std::max(lane_left_width - sl_boundary.end_l(),
                                  lane_right_width + sl_boundary.start_l());
  driving_width = std::min(lane_left_width + lane_right_width, driving_width);
  std::cout << "Driving width [" << driving_width << "]." << std::endl;
  return driving_width;
}

void ReferenceLine::ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points) {
  double s = 0.0;
  int path_points_number = 0;
  // std::vector<PathPoint> path_points;
  for (const auto& ref_point : ref_points) {
    PathPoint path_point;
    path_point.set_x(ref_point.x());
    path_point.set_y(ref_point.y());
    path_point.set_theta(ref_point.heading());
    path_point.set_kappa(ref_point.kappa());
    path_point.set_dkappa(ref_point.dkappa());

    if (!path_points_.empty()) {
      double dx = path_point.x() - path_points_.back().x();
      double dy = path_point.y() - path_points_.back().y();
      s += std::sqrt(dx * dx + dy * dy);  //这个下一帧s的计算需要注意
    }
    path_point.set_s(s);
    path_points_.push_back(std::move(path_point));
    path_points_number++;
  }
  std::cout << "ReferenceLine point number = " << path_points_number << std::endl;
}

// 新增函数，用于根据s找离散路径点上的匹配点，未测试！
size_t ReferenceLine::GetIndexFromS(const double s) const{
  // TODO(wushangzhe):未做边界判断！
  size_t index = 0;
  for(const auto path_point : path_points_) {
    if(path_point.s() > s) {
      return index;
    } else {
      index++;
    }
  }
}

// TODO(wushangzhe):用离散路径点代替原来的参考线点，是有误差的！待改进（应该取参考线平滑点），未测试！
ReferencePoint ReferenceLine::GetReferencePoint(const double s) const {
  // 正确性判断
  const auto& accumulated_s = path_points_.back().s();
  if (s < path_points_.front().s() - 1e-2) {
    std::cout << "The requested s: " << s << " < 0." << std::endl;
    return reference_points_.front();
  }
  if (s > path_points_.back().s() + 1e-2) {
    std::cout << "The requested s: " << s
          << " > reference line length: " << path_points_.back().s() << std::endl;
    return reference_points_.back();
  }

  // 获取参考线上匹配点下标
  auto interpolate_index = GetIndexFromS(s);
  return reference_points_[interpolate_index];

  // size_t index = interpolate_index;
  // size_t next_index = index + 1;
  // if (next_index >= reference_points_.size()) {
  //   next_index = reference_points_.size() - 1;
  // }

  // const auto& p0 = reference_points_[index];
  // const auto& p1 = reference_points_[next_index];

  // const double s0 = path_points_[index].s();
  // const double s1 = path_points_[next_index].s();
  // return InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index);
}

// TODO(wushangzhe)：把xy坐标投影到参考线frenet坐标系上，未进行测试！
bool ReferenceLine::XYToSL(const common::math::Vec2d& xy_point,
                           SLPoint* const sl_point) const {
  double s = 0.0;
  double l = 0.0;
  // 找到参考线上的匹配点
  PathPoint matched_point = PathMatcher::MatchToPath(
    path_points_, xy_point.x(), xy_point.y());
  // 笛卡尔转frenet
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s(), matched_point.x(), matched_point.y(),
      matched_point.theta(), xy_point.x(), xy_point.y(), &s, &l);

  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

bool ReferenceLine::SLToXY(const SLPoint& sl_point,
                           common::math::Vec2d* const xy_point) const {
  // if (map_path_.num_points() < 2) {
  //   std::cout << "The reference line has too few points." << std::endl;
  //   return false;
  // }

  const auto matched_point = GetReferencePoint(sl_point.s());
  const auto angle = common::math::Angle16::from_rad(matched_point.heading());
  xy_point->set_x(matched_point.x() - common::math::sin(angle) * sl_point.l());
  xy_point->set_y(matched_point.y() + common::math::cos(angle) * sl_point.l());
  return true;
}

common::FrenetFramePoint ReferenceLine::GetFrenetPoint(
    const common::PathPoint& path_point) const {
  if (reference_points_.empty()) {
    return common::FrenetFramePoint();
  }

  common::SLPoint sl_point;
  common::math::Vec2d xy_point(path_point.x(), path_point.y());
  XYToSL(xy_point, &sl_point);
  common::FrenetFramePoint frenet_frame_point;
  frenet_frame_point.set_s(sl_point.s());
  frenet_frame_point.set_l(sl_point.l());

  const double theta = path_point.theta();
  const double kappa = path_point.kappa();
  const double l = frenet_frame_point.l();

  ReferencePoint ref_point = GetReferencePoint(frenet_frame_point.s());

  const double theta_ref = ref_point.heading();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();

  const double dl = CartesianFrenetConverter::CalculateLateralDerivative(
      theta_ref, theta, l, kappa_ref);
  const double ddl =
      CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point.set_dl(dl);
  frenet_frame_point.set_ddl(ddl);
  return frenet_frame_point;
}

// 根据s查询高精地图，获取该路段的限速值
// 测试方便，都设置27.7
double ReferenceLine::GetSpeedLimitFromS(const double s) const {
  double speed_limit = FLAGS_planning_upper_speed_limit;
  return speed_limit;
}

// 
void ReferenceLine::AddSpeedLimit(double start_s, double end_s,
                                  double speed_limit) {
  std::vector<SpeedLimit> new_speed_limit;
  for (const auto& limit : speed_limit_) {
    if (start_s >= limit.end_s || end_s <= limit.start_s) {
      new_speed_limit.emplace_back(limit);
    } else {
      // start_s < speed_limit.end_s && end_s > speed_limit.start_s
      double min_speed = std::min(limit.speed_limit, speed_limit);
      if (start_s >= limit.start_s) {
        new_speed_limit.emplace_back(limit.start_s, start_s, min_speed);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        } else {
          new_speed_limit.emplace_back(start_s, limit.end_s, min_speed);
        }
      } else {
        new_speed_limit.emplace_back(start_s, limit.start_s, speed_limit);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(limit.start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        } else {
          new_speed_limit.emplace_back(limit.start_s, limit.end_s, min_speed);
        }
      }
      start_s = limit.end_s;
      end_s = std::max(end_s, limit.end_s);
    }
  }
  speed_limit_.clear();
  if (end_s > start_s) {
    new_speed_limit.emplace_back(start_s, end_s, speed_limit);
  }
  for (const auto& limit : new_speed_limit) {
    if (limit.start_s < limit.end_s) {
      speed_limit_.emplace_back(limit);
    }
  }
  std::sort(speed_limit_.begin(), speed_limit_.end(),
            [](const SpeedLimit& a, const SpeedLimit& b) {
              if (a.start_s != b.start_s) {
                return a.start_s < b.start_s;
              }
              if (a.end_s != b.end_s) {
                return a.end_s < b.end_s;
              }
              return a.speed_limit < b.speed_limit;
            });
}

}  // namespace planning
}  // namespace apollo

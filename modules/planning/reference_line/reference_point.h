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
 * @file reference_point.h
 **/

#pragma once

namespace apollo {
namespace planning {


class ReferencePoint{
 public:
  ReferencePoint() = default;

  ReferencePoint(const double x, const double y, const double heading, const double kappa,
                 const double dkappa);

  double x() const;
  double y() const;
  double heading() const;
  double kappa() const;
  double dkappa() const;
  std::string DebugString() const;

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double heading_ = 0.0;
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};

}  // namespace planning
}  // namespace apollo

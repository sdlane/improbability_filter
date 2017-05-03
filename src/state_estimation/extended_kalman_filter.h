// Copyright 2017 slane@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#ifndef SRC_STATE_ESTIMATION_EXTENDED_KALMAN_FILTER_H_
#define SRC_STATE_ESTIMATION_EXTENDED_KALMAN_FILTER_H_

#include <iostream>
#include <fstream>
#include "state_estimation/kalman_filter.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "state/team.h"

namespace estimation {
class ExtendedKalmanFilter : public KalmanFilter {
 public:
  ExtendedKalmanFilter(const pose_2d::Pose2Df&
observed_pose,
                       const double& timestep);
  ~ExtendedKalmanFilter();

  void Predict(const double& timestep);
  void Update(const pose_2d::Pose2Df& observation,
              const double& timestep);

  void CalculateJacobian(const float delta_t);

 private:
  Eigen::MatrixXf velocity_jacobian_;
  Eigen::MatrixXf noise_jacobian_;  // Should be const
  Eigen::MatrixXf process_covariance_; // Should be const

  Eigen::MatrixXf measurement_jacobian_; // Should be const
  Eigen::MatrixXf measurement_noise_jacobian_; // Should be const
  Eigen::MatrixXf measurement_covariance_; // Should be const

  std::ofstream logfile;
};
}  // namespace estimation

#endif  //  SRC_STATE_ESTIMATION_EXTENDED_KALMAN_FILTER_H_

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

#ifndef SRC_STATE_ESTIMATION_KALMAN_FILTER_H_
#define SRC_STATE_ESTIMATION_KALMAN_FILTER_H_

#include "math/poses_2d.h"
#include "eigen3/Eigen/Core"

namespace estimation {
class KalmanFilter {
 public:
  KalmanFilter(const pose_2d::Pose2Df& observed_pose,
               const double& timestep);
  ~KalmanFilter();

  virtual void Predict(const double& timestep) = 0;
  virtual void Update(const pose_2d::Pose2Df& observation,
                      const double& timestep) = 0;

  void GetCurrentState(pose_2d::Pose2Df* pose,
                       pose_2d::Pose2Df* velocity) const;
  void GetCurrentPose(pose_2d::Pose2Df* pose) const;
  void GetCurrentVelocity(pose_2d::Pose2Df* velocity) const;

 protected:
  // State is a vector of length 6. The variables are:
  //   X
  //   Y
  //   Theta
  //   v_radial
  //   v_tangential
  //   omega
  Eigen::VectorXf current_state;
  Eigen::MatrixXf current_covariance;

  double previous_predict_time;
  double previous_update_time;
};
}  // namespace estimation
#endif  //  SRC_STATE_ESTIMATION_KALMAN_FILTER_H_
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

#include "state_estimation/kalman_filter.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "math/poses_2d.h"

using Eigen::MatrixXf;
using Eigen::VectorXf;
using pose_2d::Pose2Df;

namespace estimation {

KalmanFilter::KalmanFilter(const Pose2Df& observed_pose,
                           const double& timestep) :
                           current_state(6), current_covariance(6,6) {
  previous_predict_time = timestep;
  previous_update_time = timestep;

  current_state(0) = observed_pose.translation.x();
  current_state(1) = observed_pose.translation.y();
  current_state(2) = observed_pose.angle;
  current_state(3) = 0;
  current_state(4) = 0;
  current_state(5) = 0;

  current_covariance = MatrixXf::Zero(3,3);
  current_covariance(1, 1) = 625;  // Standard deviation of 25 mm
  current_covariance(2, 2) = 625;  // Standard deviation of 25 mm
  current_covariance(3, 3) = 0.029929;  // Standard deviation of 0.173 rad
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::GetCurrentState(Pose2Df* pose, Pose2Df* velocity) const {
  pose->translation.x() = current_state(0);
  pose->translation.y() = current_state(1);
  pose->angle = current_state(2);
  velocity->translation.x() = current_state(3);
  velocity->translation.y() = current_state(4);
  velocity->angle = current_state(5);
}

void KalmanFilter::GetCurrentPose(Pose2Df* pose) const {
  pose->translation.x() = current_state(0);
  pose->translation.y() = current_state(1);
  pose->angle = current_state(2);
}

void KalmanFilter::GetCurrentVelocity(Pose2Df* velocity) const {
  velocity->translation.x() = current_state(3);
  velocity->translation.y() = current_state(4);
  velocity->angle = current_state(5);
}
}  // namespace estimation

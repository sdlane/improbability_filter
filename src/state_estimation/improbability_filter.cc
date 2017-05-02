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

// Model used is taken from Improbability Filtering for Rejecting False
//     Positives
//  c = cos(theta)
//  s = sin(theta)
//  dt = delta_t
//  vr = v_radial
//  vt = v_tangential
//
//   X(t) = transpose([x, y, theta, v_radial, v_tangential, omega])
//   B = [1 0 0 c*dt -s*dt 0,
//        0 1 0 s*dt c*dt 0,
//        0 0 1 0 0 dt,
//        0 0 0 0.5 0 0,
//        0 0 0 0 0.5 0,
//        0 0 0 0 0 0.5]
//   Update:  X(t+1) = B*X(t)
//
// Jacobian calculation:
//   A = [1 0 -(vr*s + vt*c)*dt c*dt -s*dt 0,
//        0 1 (vr*c - ct*s)*dt s*delta_t c*dt 0,
//        0 0 1 0 0 delta_t,
//        0 0 0 0.5 0 0,
//        0 0 0 0 0.5 0,
//        0 0 0 0 0 0.5]
// All others are 0

#include "state_estimation/improbability_filter.h"

#include <cmath>
#include "state_estimation/kalman_filter.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "math/poses_2d.h"
#include "constants/constants.h"

using Eigen::MatrixXf;
using Eigen::VectorXf;
using pose_2d::Pose2Df;
using std::sqrt;
using std::sin;
using std::cos;
using std::exp;
using std::pow;

namespace estimation {

ImprobabilityFilter::ImprobabilityFilter(
    const Pose2Df& observed_pose,
    const double& timestep):
    KalmanFilter(observed_pose, timestep),
    velocity_jacobian_(6, 6),
    noise_jacobian_(6, 3),
    process_covariance_(3, 3),
    measurement_jacobian_(3, 6),
    measurement_noise_jacobian_(3, 3),
    measurement_covariance_(3, 3) {
  velocity_jacobian_ = MatrixXf::Zero(6, 6);

  noise_jacobian_.topLeftCorner(3, 3) = MatrixXf::Zero(3, 3);
  noise_jacobian_.bottomRightCorner(3, 3) = MatrixXf::Identity(3, 3);

  process_covariance_ = MatrixXf::Zero(3, 3);
  process_covariance_(1, 1) = 4356;  // Standard deviation of 66 mm/s
  process_covariance_(2, 2) = 4356;  // Standard deviation of 66 mm/s
  process_covariance_(3, 3) = 0.04;  // Standard deviation of 0.2 rad/s

  measurement_jacobian_.topLeftCorner(3, 3) = MatrixXf::Identity(3, 3);
  measurement_jacobian_.topRightCorner(3, 3) = MatrixXf::Zero(3, 3);

  measurement_noise_jacobian_ = MatrixXf::Identity(3, 3);

  measurement_covariance_ = MatrixXf::Zero(3, 3);
  measurement_covariance_(1, 1) = 625;  // Standard deviation of 25 mm
  measurement_covariance_(2, 2) = 625;  // Standard deviation of 25 mm
  measurement_covariance_(3, 3) = 0.029929;  // Standard deviation of 0.173 rad


}


ImprobabilityFilter::~ImprobabilityFilter() {}

//   B = [1 0 0 c*dt -s*dt 0,
//        0 1 0 s*dt c*dt 0,
//        0 0 1 0 0 dt,
//        0 0 0 0.5 0 0,
//        0 0 0 0 0.5 0,
//        0 0 0 0 0 0.5]
void ImprobabilityFilter::Predict(const double& timestep) {
  MatrixXf prediction_matrix(6,6);

  const float theta = current_state(2);
  const float cosine = cos(theta);
  const float sine = sin(theta);
  const double delta_t = timestep - previous_predict_time;

  prediction_matrix.topLeftCorner(3,3) = MatrixXf::Identity(3,3);
  prediction_matrix.bottomRightCorner(3,3) = 0.5*MatrixXf::Identity(3,3);

  prediction_matrix(0,3) = cosine*delta_t;
  prediction_matrix(0,4) = -sine*delta_t;
  prediction_matrix(1,3) = sine*delta_t;
  prediction_matrix(1,4) = cosine*delta_t;
  prediction_matrix(2,5) = delta_t;

  CalculateJacobian(delta_t);

  current_state = prediction_matrix*current_state;

  current_covariance =
    velocity_jacobian_*current_covariance*velocity_jacobian_.transpose() +
    noise_jacobian_*process_covariance_*noise_jacobian_.transpose();

  previous_predict_time = timestep;
}

void ImprobabilityFilter::Update(const Pose2Df& observed_pose,
                                               const double& timestep) {
  if (timestep != previous_update_time)
    Predict(timestep);

  VectorXf observation(3);
  observation(0) = observed_pose.translation.x();
  observation(1) = observed_pose.translation.y();
  observation(2) = observed_pose.angle;

  MatrixXf kalman_gain(6,6);
  kalman_gain =
    current_covariance * measurement_jacobian_.transpose() *
    (measurement_jacobian_*current_covariance*measurement_jacobian_.transpose()
+
    measurement_noise_jacobian_ * measurement_covariance_ *
    measurement_noise_jacobian_.transpose()).inverse();

  VectorXf predicted_measurement(3);
  predicted_measurement(0) = current_state(0);
  predicted_measurement(1) = current_state(1);
  predicted_measurement(2) = current_state(2);


  MatrixXf transformed_covariance(3,3);
  transformed_covariance =
    measurement_jacobian_*current_covariance*measurement_jacobian_.transpose() +
    measurement_covariance_;

  float exponent = 0.5 *
                   (observation-measurement_jacobian_*current_state).transpose()
                   * measurement_covariance_.inverse() *
                   (observation-measurement_jacobian_*current_state);

  float observation_probability =
    1.0/pow(2.0*M_PI * transformed_covariance.norm(), (3.0/2.0)) *
    exp(exponent);

  if (observation_probability > kImprobabilityRejectionThreshold) {
    current_state += kalman_gain * (observation - predicted_measurement);

    current_covariance =
      (MatrixXf::Identity(3,3) - kalman_gain*measurement_jacobian_) *
      current_covariance;
    previous_update_time = timestep;
  }
}

//   A = [1 0 -(vr*s + vt*c)*dt c*dt -s*dt 0,
//        0 1 (vr*c - ct*s)*dt s*delta_t c*dt 0,
//        0 0 1 0 0 delta_t,
//        0 0 0 0.5 0 0,
//        0 0 0 0 0.5 0,
//        0 0 0 0 0 0.5]
void ImprobabilityFilter::CalculateJacobian(const float delta_t) {
  velocity_jacobian_ = MatrixXf::Zero(6, 6);

  const float theta = current_state(2);
  const float v_radial = current_state(3);
  const float v_tangential = current_state(4);
  const float cosine = cos(theta);
  const float sine = sin(theta);

  velocity_jacobian_.topLeftCorner(3, 3) = MatrixXf::Identity(3,3);
  velocity_jacobian_.bottomRightCorner(3, 3) = 0.5*MatrixXf::Identity(3,3);

  velocity_jacobian_(0, 3) = cosine*delta_t;
  velocity_jacobian_(0, 4) = -sine*delta_t;
  velocity_jacobian_(1, 3) = sine*delta_t;
  velocity_jacobian_(1, 4) = cosine*delta_t;
  velocity_jacobian_(2, 5) = delta_t;

  velocity_jacobian_(0, 2) = -(v_radial*sine + v_tangential*cosine) * delta_t;
  velocity_jacobian_(1, 2) = (v_radial*cosine + v_tangential*sine) * delta_t;
}

}  // namespace estimation
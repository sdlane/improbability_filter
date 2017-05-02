// Copyright 2017 kvedder@umass.edu
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
#include "soccer/kalmanupdate.h"

#include <glog/logging.h>

#include "state/team.h"
#include "constants/constants.h"
#include "util/timer.h"
#include "math/math_util.h"

#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"

using direction::Direction;
using estimation::ExtendedKalmanFilter;
using estimation::ImprobabilityFilter;
using math_util::AngleMod;
using pose_2d::Pose2Df;
using SSLVisionProto::SSL_DetectionBall;
using SSLVisionProto::SSL_DetectionFrame;
using SSLVisionProto::SSL_DetectionRobot;
using SSLVisionProto::SSL_WrapperPacket;
using std::atomic_bool;
using std::condition_variable;
using std::endl;
using std::make_pair;
using std::mutex;
using std::thread;
using std::vector;
using std::unique_lock;
using team::Team;

namespace app {

KalmanUpdate::KalmanUpdate(vector<SSL_WrapperPacket>* ssl_vision_queue,
                  mutex* ssl_vision_read_write_mutex,
                  condition_variable* ssl_vision_write_out_cv,
                  atomic_bool* ssl_vision_want_read,
                  mutex* ssl_vision_read_flag_mutex,
                  condition_variable* ssl_vision_read_block_cv,
                  mutex* ssl_vision_access_mutex,
                  mutex* ssl_vision_has_data_mutex,
                  condition_variable* ssl_vision_has_data_cv,
                  atomic_bool* ssl_vision_has_data,
                  const Direction& direction)
: ssl_vision_queue_(ssl_vision_queue),
ssl_vision_read_write_mutex_(ssl_vision_read_write_mutex),
ssl_vision_want_read_(ssl_vision_want_read),
ssl_vision_read_flag_mutex_(ssl_vision_read_flag_mutex),
ssl_vision_read_block_cv_(ssl_vision_read_block_cv),
ssl_vision_access_mutex_(ssl_vision_access_mutex),
ssl_vision_has_data_mutex_(ssl_vision_has_data_mutex),
ssl_vision_has_data_cv_(ssl_vision_has_data_cv),
ssl_vision_has_data_(ssl_vision_has_data),
direction(direction) {
  is_running_ = true;
  state_estimation_test_type = 0;
  initialized_filters = false;
}


KalmanUpdate::~KalmanUpdate() {
  // Signal the thread that we are shutting down
  is_running_ = false;

  // Clean out all potential blocks from the pipeline.
  *ssl_vision_has_data_ = true;
  *ssl_vision_want_read_ = false;

  {
    unique_lock<mutex> guard(*ssl_vision_has_data_mutex_);
    ssl_vision_has_data_cv_->notify_all();
  }
  {
    unique_lock<mutex> guard(*ssl_vision_read_write_mutex_);
    ssl_vision_read_block_cv_->notify_all();
  }

  LOG(INFO) << "Waition for shutdown of Kalman Update thread" << endl;
  update_thread_.join();
  LOG(INFO) << "Shutdown Kalman Update thread!" << endl;
}

void KalmanUpdate::Start() {
  update_thread_ = thread(&KalmanUpdate::HandleUpdate, this);
}

void KalmanUpdate::HandleUpdate() {
  LOG(INFO) << "KalmanUpdate Worker Setup!" << endl;

  vector<SSL_WrapperPacket> local_ssl_vision_queue;

  while (is_running_) {
    // Clear queues in preperation of refils.
    local_ssl_vision_queue.clear();

    SSL_DetectionFrame detection_frame_latest;
    // Handle removing and adding new bots.
    for (const SSL_WrapperPacket& wrapper_packet : local_ssl_vision_queue) {
      const SSL_DetectionFrame& detection_frame = wrapper_packet.detection();
      detection_frame_latest = wrapper_packet.detection();

      // Handle yellow robots.
      for (const SSL_DetectionRobot yellow_ssl_robot :
          detection_frame.robots_yellow()) {
        const float x = (direction == Direction::POSITIVE)
            ? yellow_ssl_robot.x() : yellow_ssl_robot.x() * -1;
        const float y = (direction == Direction::POSITIVE)
            ? yellow_ssl_robot.y() : yellow_ssl_robot.y() * -1;

        const float orientation = (yellow_ssl_robot.has_orientation())
                                    ? ((direction == Direction::POSITIVE) ?
yellow_ssl_robot.orientation(): yellow_ssl_robot.orientation() + M_PI) : 0.0f;

         if (!initialized_filters) {
           ekf = new ExtendedKalmanFilter(Pose2Df(AngleMod(orientation), x, y),
                                          detection_frame.t_capture());
           improb = new ImprobabilityFilter(
             Pose2Df(AngleMod(orientation), x, y),
             detection_frame.t_capture());
         } else {
           ekf->Update(Pose2Df(AngleMod(orientation), x, y),
                       detection_frame.t_capture());
           improb->Update(Pose2Df(AngleMod(orientation), x, y),
                          detection_frame.t_capture());
         }
      }

      // Handle blue robots.
      for (const SSL_DetectionRobot blue_ssl_robot :
          detection_frame.robots_blue()) {
        const float x = (direction == Direction::POSITIVE)
            ? blue_ssl_robot.x() : blue_ssl_robot.x() * -1;
        const float y = (direction == Direction::POSITIVE)
            ? blue_ssl_robot.y() : blue_ssl_robot.y() * -1;
        const float orientation = (blue_ssl_robot.has_orientation())
                                    ? ((direction == Direction::POSITIVE) ?
blue_ssl_robot.orientation(): blue_ssl_robot.orientation() + M_PI) : 0.0f;
        if (!blue_ssl_robot.has_robot_id()) {
          LOG(FATAL) << "Read robot lacks an ID\n";
        }
        const int robot_id = blue_ssl_robot.robot_id();

       // WorldRobot blue_world_robot(robot_id, Team::BLUE,
       //                             Pose2Df(AngleMod(orientation), x, y),
       //                             detection_frame.t_capture());
      }
    }
  }
}
}  // namespace app

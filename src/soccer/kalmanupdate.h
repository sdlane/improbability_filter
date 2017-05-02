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
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "state/direction.h"
#include "state_estimation/extended_kalman_filter.h"
#include "state_estimation/improbability_filter.h"

#include "messages_robocup_ssl_wrapper.pb.h"

#ifndef SRC_SOCCER_KALMANUPDATE_H_
#define SRC_SOCCER_KALMANUPDATE_H_

namespace app {
class KalmanUpdate {
 public:
  KalmanUpdate(std::vector<SSLVisionProto::SSL_WrapperPacket>* ssl_vision_queue,
               std::mutex* ssl_vision_read_write_mutex,
               std::condition_variable* ssl_vision_write_out_cv,
               std::atomic_bool* ssl_vision_want_read,
               std::mutex* ssl_vision_read_flag_mutex,
               std::condition_variable* ssl_vision_read_block_cv,
               std::mutex* ssl_vision_access_mutex,
               std::mutex* ssl_vision_has_data_mutex,
               std::condition_variable* ssl_vision_has_data_cv,
               std::atomic_bool* ssl_vision_has_data,
               const direction::Direction& direction);

  ~KalmanUpdate();

  void Start();

  // Used to switch to test mode for state estimation
  uint state_estimation_test_type;

 private:
  void HandleUpdate();

  std::atomic_bool is_running_;
  std::thread update_thread_;

  std::vector<SSLVisionProto::SSL_WrapperPacket>* ssl_vision_queue_;
  std::mutex* ssl_vision_read_write_mutex_;
  std::atomic_bool* ssl_vision_want_read_;
  std::mutex* ssl_vision_read_flag_mutex_;
  std::condition_variable* ssl_vision_read_block_cv_;
  std::mutex* ssl_vision_access_mutex_;
  std::mutex* ssl_vision_has_data_mutex_;
  std::condition_variable* ssl_vision_has_data_cv_;
  std::atomic_bool* ssl_vision_has_data_;

  const direction::Direction& direction;
};
}  // namespace app

#endif  // SRC_SOCCER_KALMANUPDATE_H_

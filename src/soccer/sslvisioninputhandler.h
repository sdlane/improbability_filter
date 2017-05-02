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
#include <string>
#include <thread>
#include <vector>

#include "net/netraw.h"

#include "messages_robocup_ssl_wrapper.pb.h"
#include "radio_protocol_wrapper.pb.h"

#ifndef SRC_SOCCER_SSLVISIONINPUTHANDLER_H_
#define SRC_SOCCER_SSLVISIONINPUTHANDLER_H_

namespace app {
class SSLVisionInputHandler {
 public:
  SSLVisionInputHandler(
      const std::string& input_udp_address,
      const int input_udp_port,
      const float loop_rate_hz,
      std::vector<SSLVisionProto::SSL_WrapperPacket>* ssl_vision_queue,
      std::mutex* ssl_vision_read_write_mutex,
      std::condition_variable* ssl_vision_write_out_cv,
      std::atomic_bool* ssl_vision_want_read,
      std::mutex* ssl_vision_read_flag_mutex,
      std::condition_variable* ssl_vision_read_block_cv,
      std::mutex* ssl_vision_access_mutex,
      std::mutex* ssl_vision_has_data_mutex,
      std::condition_variable* ssl_vision_has_data_cv,
      std::atomic_bool* ssl_vision_has_data);

  ~SSLVisionInputHandler();

  void Start();

 private:
  void HandleInput(const std::string udpAddress, const int udpPort);

  const std::string  input_udp_address_;
  const int     input_udp_port_;

  const float loop_rate_hz_;

  std::atomic_bool is_running_;
  std::thread input_thread_;

  std::vector<SSLVisionProto::SSL_WrapperPacket>* ssl_vision_queue_;
  std::mutex* ssl_vision_read_write_mutex_;
  std::atomic_bool* ssl_vision_want_read_;
  std::mutex* ssl_vision_read_flag_mutex_;
  std::condition_variable* ssl_vision_read_block_cv_;
  std::mutex* ssl_vision_access_mutex_;
  std::mutex* ssl_vision_has_data_mutex_;
  std::condition_variable* ssl_vision_has_data_cv_;
  std::atomic_bool* ssl_vision_has_data_;
};
}  // namespace app

#endif  // SRC_SOCCER_SSLVISIONINPUTHANDLER_H_

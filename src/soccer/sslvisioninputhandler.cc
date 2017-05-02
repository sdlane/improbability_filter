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
#include "soccer/sslvisioninputhandler.h"

#include "util/colorize.h"
#include "util/timer.h"
#include <glog/logging.h>

using colorize::ColorGreen;
using net::UDPMulticastServer;
using SSLVisionProto::SSL_WrapperPacket;
using std::atomic_bool;
using std::condition_variable;
using std::endl;
using std::lock_guard;
using std::unique_lock;
using std::mutex;
using std::string;
using std::thread;
using std::vector;

namespace app {

SSLVisionInputHandler::SSLVisionInputHandler(const string& input_udp_address,
    const int input_udp_port,
    const float loop_rate_hz,
    vector<SSL_WrapperPacket>* ssl_vision_queue,
    mutex* ssl_vision_read_write_mutex,
    condition_variable* ssl_vision_write_out_cv,
    atomic_bool* ssl_vision_want_read,
    mutex* ssl_vision_read_flag_mutex,
    condition_variable* ssl_vision_read_block_cv,
    mutex* ssl_vision_access_mutex,
    mutex* ssl_vision_has_data_mutex,
    condition_variable* ssl_vision_has_data_cv,
    atomic_bool* ssl_vision_has_data)
      : input_udp_address_(input_udp_address),
        input_udp_port_(input_udp_port),
        loop_rate_hz_(loop_rate_hz),
        ssl_vision_queue_(ssl_vision_queue),
        ssl_vision_read_write_mutex_(ssl_vision_read_write_mutex),
        ssl_vision_want_read_(ssl_vision_want_read),
        ssl_vision_read_flag_mutex_(ssl_vision_read_flag_mutex),
        ssl_vision_read_block_cv_(ssl_vision_read_block_cv),
        ssl_vision_access_mutex_(ssl_vision_access_mutex),
        ssl_vision_has_data_mutex_(ssl_vision_has_data_mutex),
        ssl_vision_has_data_cv_(ssl_vision_has_data_cv),
        ssl_vision_has_data_(ssl_vision_has_data) {
  is_running_ = true;
}


SSLVisionInputHandler::~SSLVisionInputHandler() {
  // Signal loops of the worker threads to terminate.
  is_running_ = false;

  *ssl_vision_has_data_ = false;
  *ssl_vision_want_read_ = false;
  {
    unique_lock<mutex> guard(*ssl_vision_has_data_mutex_);
    ssl_vision_has_data_cv_->notify_all();
  }
  {
    unique_lock<mutex> guard(*ssl_vision_read_write_mutex_);
    ssl_vision_read_block_cv_->notify_all();
  }

  // Blocking wait for input thread to terminate.
  input_thread_.join();
  LOG(INFO) << ColorGreen("Input Thread Closed Cleanly!") << endl;
}

void SSLVisionInputHandler::Start() {
  input_thread_ = thread(&SSLVisionInputHandler::HandleInput, this,
                        input_udp_address_, input_udp_port_);
}

void SSLVisionInputHandler::HandleInput(const string udpAddress,
                                        const int udpPort) {
  UDPMulticastServer udp_server;
  RateLoop rate_loop(loop_rate_hz_);
  if (!udp_server.Open(udpAddress, udpPort)) {
    LOG(FATAL) << "Error opening UDP port of SSLVisionInputHandler's "
        << "HandleInput thread, exiting." << endl;
  } else {
    LOG(INFO) << "Opening UDP of SSLVisionInputHandler's HandleInput thread "
        << "successful!" << endl;
  }
  CHECK(udp_server.IsOpen());

  LOG(INFO) << "SSLVisionInputHandler's HandleInput Setup!" << endl;

  // Reads SSL_WrapperPacket from the SSL_Vision system as input.
  SSL_WrapperPacket command;

  while (is_running_) {
    if (udp_server.TryReceiveProtobuf(&command)) {
      {
        unique_lock<mutex> guard(*ssl_vision_read_flag_mutex_);
        if (!ssl_vision_want_read_) {
          ssl_vision_read_block_cv_->wait(guard);
        }
      }
      {
        lock_guard<mutex> guard(*ssl_vision_access_mutex_);
        // Push back wrapper into the queue.
        ssl_vision_queue_->push_back(command);
      }
      if (!(*ssl_vision_has_data_)) {
        lock_guard<mutex> guard(*ssl_vision_has_data_mutex_);
        *ssl_vision_has_data_ = true;
        ssl_vision_has_data_cv_->notify_one();
      }
    } else {
      rate_loop.Sleep();
    }
  }

  LOG(INFO) << "Closing Input UDP Server!" << endl;
  udp_server.Close();
  LOG(INFO) << "SSLVisionInputHandler's HandleInput's Input Closed... "
      << "Exiting input thread..." << endl;
}
}  // namespace app

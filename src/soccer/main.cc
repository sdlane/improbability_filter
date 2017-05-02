// Copyright 2016 - 2017 kvedder@umass.edu
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

#include <glog/logging.h>
#include <signal.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>

#include "soccer/kalmanupdate.h"
#include "soccer/sslvisioninputhandler.h"
#include "state/direction.h"
#include "state/team.h"
#include "constants/constants.h"
#include "util/colorize.h"

#include "netraw_test_message.pb.h"

using app::KalmanUpdate;
using app::SSLVisionInputHandler;
using direction::Direction;
using SSLVisionProto::SSL_WrapperPacket;
using std::atomic_bool;
using std::condition_variable;
using std::endl;
using std::mutex;
using std::pair;
using std::string;
using std::unique_lock;
using std::vector;
using team::Team;

// Input address for Vision data from SSL Vision.
static const char kInputUDPAddress[] = DATA_STREAM_VISION_IP;
static const int kInputUDPPort = DATA_STREAM_VISION_PORT;

// Output address for Command data.
static const char kOutputUDPAddress[] = DATA_STREAM_CMD_IP;
static const int kOutputUDPPort = DATA_STREAM_CMD_PORT;

// Rate at which the thread polls for data from SSL_Vision.
static const float kReadLoopRate = 120;  // hz

// Used for handling the lifetime and eventual shutdown of the main RoboCup
// stack.
static atomic_bool shutdown_flag(false);
static mutex shutdown_mutex;
static condition_variable shutdown_access_cv;

// TODO(kvedder): Decide on better constants for CLI parse option array sizes.
const int kMaxOptions = 1000;
const int kMaxBuffer = 1000;

void SigHandler(int signo) {
  if (signo == SIGINT) {
    LOG(INFO) << "Recieved SIGINT, flagging for shutdown"
        << "\n";
    shutdown_flag = true;
    shutdown_access_cv.notify_all();
  }
}

string ToLowerCase(const char* cstr) {
  string arg = string(cstr);
  std::transform(arg.begin(), arg.end(), arg.begin(), ::tolower);
  return arg;
}


int main(int argc, char** argv) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO level logging.
  FLAGS_colorlogtostderr = 1;  // Colored logging.

  if (signal(SIGINT, SigHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGINT\n";
  }

  // Seed rand() for use throughout the codebase.
  auto seed_value = time(0);
  LOG(INFO) << "Global RNG Seed: " << seed_value << "\n";
  srand(seed_value);

  // TODO(kvedder): Look into better defaults.
  Direction direction = Direction::POSITIVE;
  Team team = Team::YELLOW;

  LOG(INFO) << "Starting RoboCup SSL Stack" << "\n";

  vector<SSL_WrapperPacket> ssl_vision_queue;

  mutex ssl_vision_read_write_mutex;
  condition_variable ssl_vision_write_out_cv;
  atomic_bool ssl_vision_want_read(false);

  mutex ssl_vision_read_flag_mutex;
  condition_variable ssl_vision_read_block_cv;

  mutex ssl_vision_access_mutex;

  mutex ssl_vision_has_data_mutex;
  condition_variable ssl_vision_has_data_cv;
  atomic_bool ssl_vision_has_data(false);

//   // Holds all information regarding the state of the world.
//   // Must only be accessed via its mutex.
//   WorldState global_world_state(team, robot_ids);
//   // Mutex protecting the internal state.
//   mutex world_state_mutex;
//   // Condition variable to alert waiting threads that the queue is not empty.
//   condition_variable world_state_cv;
//   // Bool for ensuring wakeups from world_state_cv aren't spurrious.
//   atomic_bool world_state_lock_read(false);

  {
    // Sets up the networking threads for I/O.
    // RAII takes care of ensuring everything shutsdown appropriately.
    SSLVisionInputHandler input_handler(kInputUDPAddress,
                                        kInputUDPPort,
                                        kReadLoopRate,
                                        &ssl_vision_queue,
                                        &ssl_vision_read_write_mutex,
                                        &ssl_vision_write_out_cv,
                                        &ssl_vision_want_read,
                                        &ssl_vision_read_flag_mutex,
                                        &ssl_vision_read_block_cv,
                                        &ssl_vision_access_mutex,
                                        &ssl_vision_has_data_mutex,
                                        &ssl_vision_has_data_cv,
                                        &ssl_vision_has_data);

    // Sets up the threads for Kalman update.
    // RAII takes care of ensuring everything shutsdown appropriately.
    KalmanUpdate kalman_update(&ssl_vision_queue,
                               &ssl_vision_read_write_mutex,
                               &ssl_vision_write_out_cv,
                               &ssl_vision_want_read,
                               &ssl_vision_read_flag_mutex,
                               &ssl_vision_read_block_cv,
                               &ssl_vision_access_mutex,
                               &ssl_vision_has_data_mutex,
                               &ssl_vision_has_data_cv,
                               &ssl_vision_has_data,
                               direction);

    // Start all threads.
    input_handler.Start();
    kalman_update.Start();

    // Begin waiting for SIGINT to proceed to shutdown.
    {
      // Transfer from the output queue to the local queue.
      unique_lock<mutex> guard(shutdown_mutex);

      // Block until woken up.
      // Check to make sure that the wakeup isn't spurrious.
      while (!shutdown_flag) {
        shutdown_access_cv.wait(guard);
      }
    }  // Lock loses scope here.
  }
  // Threads will go out of scope and shutdown here.

  LOG(INFO) << "Shutdown all threads" << "\n";

  // Cleanly exit the protobuf library.
  google::protobuf::ShutdownProtobufLibrary();
  LOG(INFO) << "Shutdown protobuf library!" << endl;
  // Ensures that all log files are cleanly flushed.
  google::FlushLogFiles(google::GLOG_INFO);
  LOG(INFO) << "Flushed log files!" << endl;
  LOG(INFO) << "Exiting..." << endl;

  return 0;
}

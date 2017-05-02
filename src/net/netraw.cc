// Copyright 2016 - 2017 joydeepb@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// UDP Networking Library
//
//========================================================================
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
//========================================================================

#include "src/net/netraw.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <glog/logging.h>
#include <google/protobuf/message_lite.h>

#include "src/util/helpers.h"

using google::protobuf::MessageLite;
using std::string;

namespace {
void PrintError(const string& error, const string& address, int port) {
  const string error_prompt =
      StringPrintf("Network error for %s:%d while %s",
                   address.c_str(),
                   port,
                   error.c_str());
  perror(error_prompt.c_str());
}
}  // namespace

namespace net {

UDPMulticastServer::UDPMulticastServer() :
    socket_fd_(0),
    port_number_(0),
    socket_address_ptr_(nullptr),
    receive_buffer_(nullptr) {
  receive_buffer_ = new char[kReceiveBufferSize];
}

UDPMulticastServer::~UDPMulticastServer() {
  Close();
  if (socket_address_ptr_) delete socket_address_ptr_;
  delete[] receive_buffer_;
}

bool UDPMulticastServer::Open(const string& address, int port) {
  address_ = address;
  port_number_ = port;
  // Open a UDP socket.
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    PrintError("opening UDP socket", address_, port_number_);
    socket_fd_ = 0;
    return false;
  }
  // Allow reuse of the socket.
  const u_int yes = 1;
  if (setsockopt(socket_fd_,
                 SOL_SOCKET,
                 SO_REUSEADDR,
                 &yes,
                 sizeof(yes)) < 0) {
    PrintError("setting SO_REUSEADDR", address_, port_number_);
    Close();
    return false;
  }
  // Save the multicast address for future Send() operations.
  if (socket_address_ptr_ == nullptr) {
    socket_address_ptr_ = new struct sockaddr_in;
  }
  memset(socket_address_ptr_, 0, sizeof(*socket_address_ptr_));
  socket_address_ptr_->sin_family = AF_INET;
  socket_address_ptr_->sin_addr.s_addr = inet_addr(address_.c_str());
  socket_address_ptr_->sin_port = htons(port_number_);

  // Bind the socket to the address for subsequent receives.
  if (bind(socket_fd_,
           reinterpret_cast<struct sockaddr*>(socket_address_ptr_),
           sizeof(*socket_address_ptr_)) < 0) {
    PrintError("setting bind", address_, port_number_);
    Close();
    return false;
  }

  // Request the kernel for multicast membership.
  ip_mreq multicast_request;
  multicast_request.imr_multiaddr.s_addr = inet_addr(address_.c_str());
  multicast_request.imr_interface.s_addr = htonl(INADDR_ANY);
  if (setsockopt(socket_fd_,
                  IPPROTO_IP,
                  IP_ADD_MEMBERSHIP,
                  &multicast_request,
                  sizeof(multicast_request)) < 0) {
    PrintError("adding multicast mermbership", address_, port_number_);
    Close();
    return false;
  }
  return true;
}

bool UDPMulticastServer::Send(const string& data) {
  if (socket_fd_ == 0) return false;
  const int bytes_sent =
      sendto(socket_fd_,
             data.data(),
             data.size(),
             0,
             reinterpret_cast<struct sockaddr*>(socket_address_ptr_),
             sizeof(*socket_address_ptr_));
  if (bytes_sent < 0 || bytes_sent != static_cast<int>(data.size())) {
    const string error =
        StringPrintf("sending %d bytes to %s:%d, only %d sent\n",
                     static_cast<int>(data.size()),
                     address_.c_str(),
                     port_number_,
                     bytes_sent);
    PrintError(error, address_, port_number_);
    return false;
  }
  return true;
}

bool UDPMulticastServer::SendProtobuf(const MessageLite& message) {
  const string data = message.SerializeAsString();
  return (Send(data));
}

void UDPMulticastServer::Close() {
  if (socket_fd_ != 0) {
    close(socket_fd_);
    socket_fd_ = 0;
  }
}

bool UDPMulticastServer::IsOpen() const {
  return (socket_fd_ > 0);
}

bool UDPMulticastServer::RawReceive(int flags, int* received_bytes) {
  sockaddr_in source_address;
  memset(reinterpret_cast<void*>(&source_address), 0, sizeof(source_address));
  socklen_t source_address_length = sizeof(source_address);
  *received_bytes = recvfrom(
      socket_fd_,
      reinterpret_cast<void*>(receive_buffer_),
      kReceiveBufferSize,
      flags,
      reinterpret_cast<sockaddr*>(socket_address_ptr_),
      &source_address_length);
  if (*received_bytes < 0 && flags == 0) {
    PrintError("receiving data", address_, port_number_);
    return false;
  }
  return true;
}

bool UDPMulticastServer::Receive(string* data) {
  DCHECK(data != nullptr);
  int received_bytes = 0;
  if (!RawReceive(0, &received_bytes)) {
    return false;
  }
  data->assign(receive_buffer_, received_bytes);
  return true;
}

bool UDPMulticastServer::ReceiveProtobuf(MessageLite* message) {
  DCHECK(message != nullptr);
  int received_bytes = 0;
  if (!RawReceive(0, &received_bytes)) {
    return false;
  }
  message->ParseFromArray(receive_buffer_, received_bytes);
  return true;
}

bool UDPMulticastServer::TryReceive(string* data) {
  DCHECK(data != nullptr);
  int received_bytes = 0;
  if (!RawReceive(MSG_DONTWAIT, &received_bytes)) {
    return false;
  } else if (received_bytes == 0) {
    // Not an error, there just isn't any data available to read right now.
    return false;
  }
  data->assign(receive_buffer_, received_bytes);
  return true;
}

bool UDPMulticastServer::TryReceiveProtobuf(MessageLite* message) {
  DCHECK(message != nullptr);
  int received_bytes = 0;
  errno = 0;
  int return_val = RawReceive(MSG_DONTWAIT, &received_bytes);
  int errsv = errno;
  if (!return_val) {
    return false;
  } else if (received_bytes == 0 || errsv == EAGAIN) {
    // Not an error, there just isn't any data available to read right now.
    return false;
  } else if (received_bytes < 0) {
    // Reading failed, logging with errno.
    LOG(WARNING) << "Reading in TryRecieveProtobuf() failed with errno: "
        << errsv << " (" << strerror(errsv) << ")!\n";
    return false;
  }
  message->ParseFromArray(receive_buffer_, received_bytes);
  return true;
}

}  // namespace net

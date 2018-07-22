/*
 * Authors: Kofi and Isabella
 * Organisation: HYPED
 * Date: 1. April 2018
 * Description:
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef BEAGLEBONE_BLACK_COMMUNICATIONS_COMMUNICATIONS_HPP_
#define BEAGLEBONE_BLACK_COMMUNICATIONS_COMMUNICATIONS_HPP_

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <cstdio>
#include <string>
#include <cstring>
#include <cstdlib>

#include "data/data.hpp"
#include "utils/logger.hpp"

namespace hyped {

using data::NavigationType;
using utils::Logger;

namespace communications {

class Communications
{
 public:
  explicit Communications(Logger& log, const char* ip, int port_no);
  ~Communications();
  /**
   * @brief Sends data to server.
   *
   * @pararm[in] message which contains with a command code, a value and a newline
   */
  int sendData(std::string message);
  /**
   * @brief Reads run length data from Server
   *
   * @return float Returns length of track in m. Buffer assumes test track will be < 1250m.
   */
  int receiveRunLength();
  /**
   * @brief Reads data from server.
   *
   * @return int Returns command code (1 = STOP, 2 = KILL POWER, 3 = LAUNCH)
   */
  int receiveMessage();
  /**
   * @brief Reads data from server.
   *
   * @return bool Returns true if connection is established
   */
  bool isConnected();

 private:
  Logger&     log_;
  data::Data& data_;
  bool is_connected_;
  int  sockfd_;
  char buffer_[256];
};

}}  //  namespace hyped::communications

#endif  // BEAGLEBONE_BLACK_COMMUNICATIONS_COMMUNICATIONS_HPP_

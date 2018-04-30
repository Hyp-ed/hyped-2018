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
#include <netdb.h>
#include <cstdio>
#include <string>
#include <cstring>
#include <cstdlib>

#include "data/data.hpp"
#include "utils/logger.hpp"
#include "data/data.hpp"


namespace hyped {

using data::NavigationType;
using utils::Logger;

namespace communications {

class Communications
{
 public:
  explicit Communications(Logger& log, const char* ip, int portNo);
  ~Communications();
  int sendDistance(NavigationType distance);    // CMD01
  int sendVelocity(NavigationType speed);       // CMD02
  int sendAcceleration(NavigationType accel);   // CMD03
  int sendStripeCount(int stripes);             // CMD04
  int sendRpmFl(float rpmfl);                   // CMD05
  int sendRpmFr(float rpmfr);                   // CMD06
  int sendRpmBl(float rpmBl);                   // CMD07
  int sendRpmBr(float rpmBr);                   // CMD08
  int sendData(std::string message);
  int receiveMessage();

 private:
  int sockfd_, n_;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  char buffer[256];
  Logger& log_;
};

}}  //  namespace hyped::communications

#endif  // BEAGLEBONE_BLACK_COMMUNICATIONS_COMMUNICATIONS_HPP_

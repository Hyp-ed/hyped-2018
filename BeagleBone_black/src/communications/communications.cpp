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


#include "communications.hpp"

#include <string>

namespace hyped {

namespace communications {

Communications::Communications(Logger& log, const char* ip, int portNo)
    : log_(log)
{
  log_.INFO("COMN", "BaseCommunicator initialised.");
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);   // socket(int domain, int type, int protocol)

  if (sockfd_ < 0) {
    log_.ERR("COMN", "CANNOT OPEN SOCKET.");
  }

  server = gethostbyname(ip);

  if (server == NULL) {
    log_.ERR("COMN", "INCORRECT BASE-STATION IP, OR BASE-STATION S/W NOT RUNNING.");
  }

  memset(&serv_addr, '\0', sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;   // server byte order
  // memcpy(server->h_addr, &serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(portNo);

  if (inet_pton(AF_INET, ip, &serv_addr.sin_addr) <= 0) {
    log_.ERR("COMN", "Invalid address.\n");
  }

  if (connect(sockfd_, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    log_.ERR("COMN", "CANNOT ESTABLISH CONNECTION TO BASE-STATION.");
  }

  log_.INFO("COMN", "TCP/IP connection established.");
}

Communications::~Communications()
{
  close(sockfd_);
}

int Communications::sendData(std::string message)
{
  // Incoming strings should be terminated by "...\n".
  memset(buffer, '\0', 256);
  const char *data = message.c_str();
  n_ = write(sockfd_, data, message.length());  // ‘_size_t write(int, const void*, size_t)’

  if (n_ < 0) {
    log_.ERR("COMN", "CANNOT READ FROM SOCKET.\n");
  }

  return atoi(buffer);
}

int Communications::receiveMessage()
{
  n_ = read(sockfd_, buffer, 255);

  if (n_ < 0) {
    log_.ERR("COMN", "CANNOT READ FROM SOCKET.\n");
    command_ = 1;
  }

  command_ = atoi(buffer);

  switch (command_) {
    case 1:
      log_.INFO("COMN", "Received 1 (STOP)");  // STOP
      break;
    case 2:
      log_.INFO("COMN", "Received 2 (LAUNCH)");  // LAUNCH
      break;
    case 3:
      log_.INFO("COMN", "Received 3 (RESET)");  // RESET
      break;
  }

  return command_;
}
}}  // namespace hyped::communcations

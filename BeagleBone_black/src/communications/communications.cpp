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

Communications::Communications(Logger& log, const char* ip, int port_no)
    : log_(log),
      data_(data::Data::getInstance()),
      is_connected_(false)
{
  log_.INFO("COMN", "BaseCommunicator initialised.");
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);   // socket(int domain, int type, int protocol)
  struct sockaddr_in serv_addr;
  struct hostent *server;

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
  serv_addr.sin_port = htons(port_no);

  if (inet_pton(AF_INET, ip, &serv_addr.sin_addr) <= 0) {
    log_.ERR("COMN", "INVALID ADDRESS.\n");
  }

  if (connect(sockfd_, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    // if connect does not complete successfully it returns -1
    log_.ERR("COMN", "CANNOT ESTABLISH CONNECTION TO BASE-STATION.");
  } else {
    log_.INFO("COMN", "TCP/IP connection established.");
    is_connected_ = true;
  }
}

Communications::~Communications()
{
  close(sockfd_);
}

int Communications::sendData(std::string message)
{
  if (sockfd_ < 0) {
    return 1;
  }

  // Incoming strings should be terminated by "...\n".
  memset(buffer_, '\0', 256);
  const char *data = message.c_str();
  int n = write(sockfd_, data, message.length());  // ‘_size_t write(int, const void*, size_t)’

  if (n < 0) {
    log_.ERR("COMN", "CANNOT WRITE TO SOCKET.\n");
    sockfd_ = -1;
    return 1;
  }

  return atoi(buffer_);
}

int Communications::receiveRunLength()
{
  if (sockfd_ < 0) {
    return 1;
  }

  int n = read(sockfd_, buffer_, 255);

  if (n < 0) {
    log_.ERR("COMN", "CANNOT READ FROM SOCKET.\n");
    sockfd_ = -1;
    return 1;  // TODO(Kofi): Need a mechanism for lost detection here (?)
  }

  int run_length = atoi(buffer_);
  log_.INFO("COMN", "Received track length of %.3fm", static_cast<float>(run_length));

  return run_length;
}

int Communications::receiveMessage()
{
  if (sockfd_ < 0) {
    return 7;  // 7 indicates connection lost
  }

  int n = read(sockfd_, buffer_, 255);

  if (n < 0) {
    log_.ERR("COMN", "CANNOT READ FROM SOCKET.\n");
    sockfd_ = -1;
    return 7;
  }

  int command = buffer_[0]-'0';

  return command;
}

bool Communications::isConnected()
{
  return is_connected_;
}

}}  // namespace hyped::communcations

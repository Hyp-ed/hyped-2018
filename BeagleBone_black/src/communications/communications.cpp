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

#include <sstream>
#include <string>

namespace hyped {
namespace communications {

std::string defaultIP = "localhost";
std::string ipAddress = defaultIP;

Communications::Communications(Logger& log): log_(log)
{
  log_.INFO("COMMUNICATIONS", "BaseCommunicator initialised");
}

Communications::Communications(Logger& log, char* ip_): log_(log)
{
  log_.INFO("COMMUNICATIONS", "BaseCommunicator initialised");
  ipAddress = ip;
}

bool Communications::setUp()
{
  portNo_ = 5695;
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);   // socket(int domain, int type, int protocol)

  if (sockfd < 0) {
    log_.ERR("COMMUNICATIONS", "CANNOT OPEN SOCKET.");
    return false;
  }

  server = gethostbyname(ipAddress);

  if (server == NULL) {
    log_.ERR("COMMUNICATIONS", "INCORRECT BASE-STATION IP, OR BASE-STATION S/W NOT RUNNING.");
    return false;
  }

  bzero(&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;   // server byte order
  bcopy(server->h_addr, &serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(portNo_);

  if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    log_.ERR("COMMUNICATIONS", "CANNOT ESTABLISH CONNECTION TO BASE-STATION.");
    return false;
  }

  // n = read(sockfd, buffer, 255);
  return true;
}

Communications::~Communications()
{
  close(sockfd_);
}

int Communications::sendDistance(float distance)
{
  stringstream ss(stringstream::in | stringstream::out);
  ss << distance;

  return sendData("CMD01" + ss.str() + "\n");
}

int Communications::sendVelocity(float speed)
{
  stringstream ss(stringstream::in | stringstream::out);
  ss << speed;

  return sendData("CMD02" + ss.str() + "\n");
}

int Communications::sendAcceleration(float accel)
{
  stringstream ss(stringstream::in | stringstream::out);
  ss << accel;

  return sendData("CMD03" + ss.str() + "\n");
}

int Communications::sendStripeCount(int stripes)
{
  stringstream ss(stringstream::in | stringstream::out);
  ss << stripes;

  return sendData("CMD04" + ss.str() + "\n");
}

int Communications::sendRpmFl(float rpmfl)
{
  stringstream ss(stringstream::in | stringstream::out);
  ss << rpmfl;

  return sendData("CMD05" + ss.str() + "\n");
}

int Communications::sendRpmFr(float rpmfr)
{
  stringstream ss(stringstream::in | stringstream::out);
  ss << rpmfr;

  return sendData("CMD06" + ss.str() + "\n");
}

int Communications::sendRpmBl(float rpmbl)
{
  stringstream ss(stringstream::in | stringstream::out);
  ss << rpmbl;

  return sendData("CMD07" + ss.str() + "\n");
}

int Communications::sendRpmBr(float rpmbr)
{
  stringstream ss(stringstream::in | stringstream::out);
  ss << rpmbr;

  return sendData("CMD08" + ss.str() + "\n");
}

int Communications::sendData(string message)
{
  // Incoming strings should be terminated by "...\n".
  bzero(buffer, 256);
  std::string data_ = message;
  n_ = write(sockfd_, data_, strlen(data_));
  if (n_ < 0) log_.ERR("COMMUNICATIONS", "CANNOT WRITE TO SOCKET.\n");
  n_ = read(sockfd_, buffer, 255);
  if (n_ < 0) log_.ERR("COMMUNICATIONS", "CANNOT READ FROM SOCKET.\n");

  return atoi(buffer);
}

void Communications::receiveMessage()
{
  n_ = read(sockfd_, buffer, 255);
  int command = atoi(buffer);

  switch (command) {
    case 1:
      log_.INFO("COMMUNICATIONS", "Received 1");
      break;
    case 2:
      log_.INFO("COMMUNICATIONS", "Received 2");
      break;
    case 3:
      log_.INFO("COMMUNICATIONS", "Received 3");
      break;
  }
}
}}  // namespace hyped::communcations

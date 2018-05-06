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

using data::Communications;

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
  memcpy(server->h_addr, &serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(portNo);

  if (connect(sockfd_, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    log_.ERR("COMN", "CANNOT ESTABLISH CONNECTION TO BASE-STATION.");
  }

  log_.INFO("COMN", "TCP/IP connection established.");
}

Communications::~Communications()
{
  close(sockfd_);
}

int Communications::sendDistance(float distance)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << distance;

  return sendData("CMD01" + ss.str() + "\n");
}

int Communications::sendVelocity(float speed)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << speed;

  return sendData("CMD02" + ss.str() + "\n");
}

int Communications::sendAcceleration(float accel)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << accel;

  return sendData("CMD03" + ss.str() + "\n");
}

int Communications::sendStripeCount(int stripes)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << stripes;

  return sendData("CMD04" + ss.str() + "\n");
}

int Communications::sendRpmFl(float rpmfl)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << rpmfl;

  return sendData("CMD05" + ss.str() + "\n");
}

int Communications::sendRpmFr(float rpmfr)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << rpmfr;

  return sendData("CMD06" + ss.str() + "\n");
}

int Communications::sendRpmBl(float rpmbl)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << rpmbl;

  return sendData("CMD07" + ss.str() + "\n");
}

int Communications::sendRpmBr(float rpmbr)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << rpmbr;

  return sendData("CMD08" + ss.str() + "\n");
}

int Communications::sendData(std::string message)
{
  // Incoming strings should be terminated by "...\n".
  memset(buffer, '\0', 256);
  const char *data_ = message.c_str();  // cannot use string because strlen requies char*
  n_ = write(sockfd_, data_, strlen(data_));
  if (n_ < 0) log_.ERR("COMN", "CANNOT WRITE TO SOCKET.\n");
  n_ = read(sockfd_, buffer, 255);
  if (n_ < 0) log_.ERR("COMN", "CANNOT READ FROM SOCKET.\n");

  return atoi(buffer);
}

int Communications::receiveMessage()
{
  n_ = read(sockfd_, buffer, 255);
  int command = atoi(buffer);
//  data::Communications comms_data;

  switch (command) {
    case 1:
      log_.INFO("COMN", "Received 1");  // STOP
//      comms_data.stopCommand = true;
//      data_.setCommunicationsData(comms_data);
      break;
    case 2:
      log_.INFO("COMN", "Received 2");  // KILL POWER
//      comms_data.killPowerCommand = true;
//      data_.setCommunicationsData(comms_data);
      break;
    case 3:
      log_.INFO("CMN", "Received 3");  // LAUCNH
//      comms_data.launch = true;
//      data_.setCommunicationsData(comms_data);
      break;
  }

  return command;
}
}}  // namespace hyped::communcations

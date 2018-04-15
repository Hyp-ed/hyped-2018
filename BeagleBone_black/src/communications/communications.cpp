/*
 * Authors: Kofi
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

using namespace std;

namespace hyped {
namespace communications {

int sockfd, portNo, n;
struct sockaddr_in serv_addr;
struct hostent *server;
char buffer[256];
const char* defaultIP = "localhost";
char* ipAddress = const_cast<char*>(defaultIP);

// TODO(Isabella): implement logger for Communications class
Communications::Communications()
{ /* EMPTY */ }

Communications::Communications(char* ip)
{
  ipAddress = ip;
}

bool Communications::setUp()
{
  portNo = 5695;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);   // socket(int domain, int type, int protocol)

  if (sockfd < 0) {
    printf("ERROR: CANNOT OPEN SOCKET.\n");
    return false;
  }

  server = gethostbyname(ipAddress);

  if (server == NULL) {
    fprintf(stderr, "ERROR: INCORRECT BASE-STATION IP, OR BASE-STATION S/W NOT RUNNING.\n");
    return false;
  }

  bzero(&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;   // server byte order
  bcopy(server->h_addr, &serv_addr.sin_addr.s_addr, server->h_length);
  serv_addr.sin_port = htons(portNo);

  if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    printf("ERROR: CANNOT ESTABLISH CONNECTION TO BASE-STATION.\n");
    return false;
  }

  // n = read(sockfd, buffer, 255);
  return true;
}

Communications::~Communications()
{
  close(sockfd);
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
  const char *data = message.c_str();
  n = write(sockfd, data, strlen(data));
  if (n < 0) printf("ERROR: CANNOT WRITE TO SOCKET.");
  n = read(sockfd, buffer, 255);
  if (n < 0) printf("ERROR: CANNOT READ FROM SOCKET.");

  return atoi(buffer);
}

/*
// Thread must be declared and joined within calling code.
void Communications :: receiverThread()
{
  while (true)
  {
    n = read(sockfd, buffer, 255);
    int command = atoi(buffer);

    switch (command)
    {
      case 1:
        printf("ECHO message (1) received.\n");
        // Do nothing, 1 represents echo message
        break;
      case 2:
        printf("READY TO LAUNCH\n");
    }
}
}
*/
}}  // namespace hyped::communcations

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


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <iostream>
using namespace std;

namespace hyped {
namespace communications {

class BaseCommunicator
{
 private:
  int sockfd, portNo, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  char buffer[256];
  // char* ipAddress;

 public:
  BaseCommunicator();
  explicit BaseCommunicator(char* ip);
  bool setUp();
  ~BaseCommunicator();
  int sendDistance(float distance);   // CMD01
  int sendVelocity(float speed);      // CMD02
  int sendAcceleration(float accel);  // CMD03
  int sendStripeCount(int stripes);   // CMD04
  int sendRpmFl(float rpmfl);         // CMD05
  int sendRpmFr(float rpmfr);         // CMD06
  int sendRpmBl(float rpmBl);         // CMD07
  int sendRpmBr(float rpmBr);         // CMD08
  int sendData(string message);
  // void receiverThread();
};

}}

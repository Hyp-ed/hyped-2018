/*
 * Author: Isabella Chan
 * Organisation: HYPED
 * Date: 30/03/18
 * Description: Demo for communications module
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

#include "communications/main.hpp"
#include "utils/logger.hpp"

using namespace hyped::communications;

int main()
{
  const char* ipAddress = "localhost";
  int portNo = 5695;
  hyped::utils::Logger log(true, 1);
  hyped::communications::Communications baseCommunicator(log, ipAddress, portNo);
  baseCommunicator.sendVelocity(6546);
  baseCommunicator.sendDistance(564);
  baseCommunicator.sendAcceleration(584);
  baseCommunicator.sendStripeCount(964);

  return 0;
}

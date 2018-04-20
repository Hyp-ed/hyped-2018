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

#include "communications/main.hpp"

namespace hyped {
namespace communications {

Main::Main(uint8_t id, Logger& log) : Thread(id, log)
{
  // To use IP address: Communications((char *) "127.0.0.1");
  baseCommunicator = new Communications();
  baseCommunicator->setUp();
  // TODO(kofi): start receiverThread here
}

void Main::run()
{
  while (1) {
    nav = data.getNavigationData();
    mtr = data.getMotorData();
    sensors_ = data.getSensorsData();
    baseCommunicator->sendDistance(nav.distance);
    baseCommunicator->sendVelocity(nav.velocity);
    baseCommunicator->sendAcceleration(nav.acceleration);
    baseCommunicator->sendStripeCount(sensors_.stripe_cnt.value);
    baseCommunicator->sendRpmFl(mtr.rpm_FL);
    baseCommunicator->sendRpmFr(mtr.rpm_FR);
    baseCommunicator->sendRpmBl(mtr.rpm_BL);
    baseCommunicator->sendRpmBr(mtr.rpm_BR);
  }
}

}}

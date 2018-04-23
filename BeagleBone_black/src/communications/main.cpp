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

#include "communications/main.hpp"

namespace hyped {
namespace communications {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log)
{
  const char* ipAddress = "localhost";
  baseCommunicator_ = new Communications(log, ipAddress);
}

void Main::run()
{
  ReceiverThread* receiverThread = new ReceiverThread(baseCommunicator_);
  receiverThread->start();

  while (1) {
    nav_ = data_.getNavigationData();
    mtr_ = data_.getMotorData();
    baseCommunicator_->sendDistance(nav_.distance);
    baseCommunicator_->sendVelocity(nav_.velocity);
    baseCommunicator_->sendAcceleration(nav_.acceleration);
    // baseCommunicator_->sendStripeCount(nav.stripe_count);
    baseCommunicator_->sendRpmFl(mtr_.rpm_FL);
    baseCommunicator_->sendRpmFr(mtr_.rpm_FR);
    baseCommunicator_->sendRpmBl(mtr_.rpm_BL);
    baseCommunicator_->sendRpmBr(mtr_.rpm_BR);
  }

  receiverThread->join();
  delete receiverThread;
}

}}

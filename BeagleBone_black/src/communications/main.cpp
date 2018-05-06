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
  int portNo = 5695;
  baseCommunicator_ = new Communications(log, ipAddress, portNo);
}

void Main::run()
{
  ReceiverThread* receiverThread = new ReceiverThread(baseCommunicator_);
  receiverThread->start();

  while (1) {
    nav_ = data_.getNavigationData();
    mtr_ = data_.getMotorData();
    sns_ = data_.getSensorsData();
    baseCommunicator_->sendDistance(nav_.distance);
    baseCommunicator_->sendVelocity(nav_.velocity);
    baseCommunicator_->sendAcceleration(nav_.acceleration);
    baseCommunicator_->sendStripeCount(sns_.stripe_count.value);
    baseCommunicator_->sendRpmFl(mtr_.motor_velocity_1);
    baseCommunicator_->sendRpmFr(mtr_.motor_velocity_2);
    baseCommunicator_->sendRpmBl(mtr_.motor_velocity_3);
    baseCommunicator_->sendRpmBr(mtr_.motor_velocity_4);
  }

  receiverThread->join();
  delete receiverThread;
}

}}

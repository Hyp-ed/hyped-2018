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

#include <sstream>

namespace hyped {
namespace communications {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log)
{
  const char* ipAddress = "localhost";
  int portNo = 5695;
  baseCommunicator_ = new Communications(log, ipAddress, portNo);
}

int Main::sendDistance(NavigationType distance)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << distance;

  return baseCommunicator_->sendData("CMD01" + ss.str() + "\n");
}

int Main::sendVelocity(NavigationType speed)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << speed;

  return baseCommunicator_->sendData("CMD02" + ss.str() + "\n");
}

int Main::sendAcceleration(NavigationType accel)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << accel;

  return baseCommunicator_->sendData("CMD03" + ss.str() + "\n");
}

int Main::sendStripeCount(int stripes)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << stripes;

  return baseCommunicator_->sendData("CMD04" + ss.str() + "\n");
}

int Main::sendRpmFl(float rpmfl)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << rpmfl;

  return baseCommunicator_->sendData("CMD05" + ss.str() + "\n");
}

int Main::sendRpmFr(float rpmfr)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << rpmfr;

  return baseCommunicator_->sendData("CMD06" + ss.str() + "\n");
}

int Main::sendRpmBl(float rpmbl)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << rpmbl;

  return baseCommunicator_->sendData("CMD07" + ss.str() + "\n");
}

int Main::sendRpmBr(float rpmbr)
{
  std::stringstream ss(std::stringstream::in | std::stringstream::out);
  ss << rpmbr;

  return baseCommunicator_->sendData("CMD08" + ss.str() + "\n");
}

void Main::run()
{
  ReceiverThread* receiverThread = new ReceiverThread(baseCommunicator_);
  receiverThread->start();

  while (1) {
    nav_ = data_.getNavigationData();
    mtr_ = data_.getMotorData();
    sns_ = data_.getSensorsData();
    sendDistance(nav_.distance);
    sendVelocity(nav_.velocity);
    sendAcceleration(nav_.acceleration);
    sendStripeCount(sns_.stripe_count.value);
    sendRpmFl(mtr_.motor_velocity_1);
    sendRpmFr(mtr_.motor_velocity_2);
    sendRpmBl(mtr_.motor_velocity_3);
    sendRpmBr(mtr_.motor_velocity_4);
  }

  receiverThread->join();
  delete receiverThread;
}

}}

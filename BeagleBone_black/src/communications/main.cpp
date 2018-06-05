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

using data::State;
using data::Battery;

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
  return baseCommunicator_->sendData("CMD01" + std::to_string(distance) + "\n");
}

int Main::sendVelocity(NavigationType speed)
{
  return baseCommunicator_->sendData("CMD02" + std::to_string(speed) + "\n");
}

int Main::sendAcceleration(NavigationType accel)
{
  return baseCommunicator_->sendData("CMD03" + std::to_string(accel) + "\n");
}

int Main::sendStripeCount(int stripes)
{
  return baseCommunicator_->sendData("CMD04" + std::to_string(stripes) + "\n");
}

int Main::sendRpmFl(float rpmfl)
{
  return baseCommunicator_->sendData("CMD05" + std::to_string(rpmfl) + "\n");
}

int Main::sendRpmFr(float rpmfr)
{
  return baseCommunicator_->sendData("CMD06" + std::to_string(rpmfr) + "\n");
}

int Main::sendRpmBl(float rpmbl)
{
  return baseCommunicator_->sendData("CMD07" + std::to_string(rpmbl) + "\n");
}

int Main::sendRpmBr(float rpmbr)
{
  return baseCommunicator_->sendData("CMD08" + std::to_string(rpmbr) + "\n");
}

int Main::sendState(State state)
{
  switch (state) {
    case data::kIdle             : stateCode_ = 0; break;
    case data::kAccelerating     : stateCode_ = 1; break;
    case data::kDecelerating     : stateCode_ = 2; break;
    case data::kEmergencyBraking : stateCode_ = 3; break;
    case data::kRunComplete      : stateCode_ = 4; break;
    case data::kFailureStopped   : stateCode_ = 5; break;
    case data::kExiting          : stateCode_ = 6; break;
    case data::kFinished         : stateCode_ = 7; break;
    case data::kInvalid          : stateCode_ = 8; break;
    default: break;
  }

  return baseCommunicator_->sendData("CMD09" + std::to_string(stateCode_) + "\n");
}

int Main::sendHpVoltage(Battery hpBattery)
{
  return baseCommunicator_->sendData("CMD10" + std::to_string(hpBattery.voltage) + "\n");
}

int Main::sendHpTemperature(Battery hpBattery)
{
  return baseCommunicator_->sendData("CMD11" + std::to_string(hpBattery.temperature) + "\n");
}

int Main::sendHpVoltage1(Battery hpBattery1)
{
  return baseCommunicator_->sendData("CMD12" + std::to_string(hpBattery1.voltage) + "\n");
}

int Main::sendHpTemperature1(Battery hpBattery1)
{
  return baseCommunicator_->sendData("CMD13" + std::to_string(hpBattery1.temperature) + "\n");
}

void Main::run()
{
  ReceiverThread* receiverThread = new ReceiverThread(baseCommunicator_);
  receiverThread->start();

  while (1) {
    nav_ = data_.getNavigationData();
    mtr_ = data_.getMotorData();
    sns_ = data_.getSensorsData();
    stm_ = data_.getStateMachineData();
    bat_ = data_.getBatteryData();
    sendDistance(nav_.distance);
    sendVelocity(nav_.velocity);
    sendAcceleration(nav_.acceleration);
    sendStripeCount(sns_.stripe_count.value);
    sendRpmFl(mtr_.motor_velocity_1);
    sendRpmFr(mtr_.motor_velocity_2);
    sendRpmBl(mtr_.motor_velocity_3);
    sendRpmBr(mtr_.motor_velocity_4);
    sendState(stm_.current_state);
    sendHpVoltage(bat_.high_power_batteries.at(0));
    sendHpTemperature(bat_.high_power_batteries.at(0));
    sendHpVoltage1(bat_.high_power_batteries.at(1));
    sendHpTemperature1(bat_.high_power_batteries.at(1));
  }

  receiverThread->join();
  delete receiverThread;
}

}}

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
  const char* ipAddress = "127.0.0.1";
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

int Main::sendRpmFl(float rpmfl)
{
  return baseCommunicator_->sendData("CMD04" + std::to_string(rpmfl) + "\n");
}

int Main::sendRpmFr(float rpmfr)
{
  return baseCommunicator_->sendData("CMD05" + std::to_string(rpmfr) + "\n");
}

int Main::sendRpmBl(float rpmbl)
{
  return baseCommunicator_->sendData("CMD06" + std::to_string(rpmbl) + "\n");
}

int Main::sendRpmBr(float rpmbr)
{
  return baseCommunicator_->sendData("CMD07" + std::to_string(rpmbr) + "\n");
}

int Main::sendState(State state)
{
  switch (state) {
    case data::kIdle             : stateCode_ = 0; break;
    case data::kCalibrating      : stateCode_ = 1; break;
    case data::kReady            : stateCode_ = 2; break;
    case data::kAccelerating     : stateCode_ = 3; break;
    case data::kDecelerating     : stateCode_ = 4; break;
    case data::kEmergencyBraking : stateCode_ = 5; break;
    case data::kRunComplete      : stateCode_ = 6; break;
    case data::kFailureStopped   : stateCode_ = 7; break;
    case data::kExiting          : stateCode_ = 8; break;
    case data::kFinished         : stateCode_ = 9; break;
    default: break;
  }

  return baseCommunicator_->sendData("CMD08" + std::to_string(stateCode_) + "\n");
}

int Main::sendHpVoltage(Battery hpBattery)
{
  // Convert voltage reading units from mV to V
  return baseCommunicator_->sendData("CMD09" + std::to_string(hpBattery.voltage / 1000.0) + "\n");
}

int Main::sendHpTemperature(Battery hpBattery)
{
  return baseCommunicator_->sendData("CMD10" + std::to_string(hpBattery.temperature) + "\n");
}

int Main::sendHpCharge(Battery hpBattery)
{
  return baseCommunicator_->sendData("CMD11" + std::to_string(hpBattery.charge) + "\n");
}

int Main::sendHpVoltage1(Battery hpBattery1)
{
  // Convert voltage reading units from mV to V
  return baseCommunicator_->sendData("CMD12" + std::to_string(hpBattery1.voltage / 1000.0) + "\n");
}

int Main::sendHpTemperature1(Battery hpBattery1)
{
  return baseCommunicator_->sendData("CMD13" + std::to_string(hpBattery1.temperature) + "\n");
}

int Main::sendHpCharge1(Battery hpBattery1)
{
  return baseCommunicator_->sendData("CMD14" + std::to_string(hpBattery1.charge) + "\n");
}

int Main::sendLpCharge(Battery lpBattery)
{
  return baseCommunicator_->sendData("CMD15" + std::to_string(lpBattery.charge) + "\n");
}

int Main::sendLpCharge1(Battery lpBattery1)
{
  return baseCommunicator_->sendData("CMD16" + std::to_string(lpBattery1.charge) + "\n");
}

int Main::sendTorqueFr(float torquefr)
{
  return baseCommunicator_->sendData("CMD17" + std::to_string(torquefr) + "\n");
}

int Main::sendTorqueFl(float torquefl)
{
  return baseCommunicator_->sendData("CMD18" + std::to_string(torquefl) + "\n");
}

int Main::sendTorqueBr(float torquebr)
{
  return baseCommunicator_->sendData("CMD19" + std::to_string(torquebr) + "\n");
}

int Main::sendTorqueBl(float torquebl)
{
  return baseCommunicator_->sendData("CMD20" + std::to_string(torquebl) + "\n");
}

int Main::sendImu1(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD2101\n");
  }

  return baseCommunicator_->sendData("CMD2100\n");
}

int Main::sendImu2(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD2201\n");
  }

  return baseCommunicator_->sendData("CMD2200\n");
}

int Main::sendImu3(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD2301\n");
  }

  return baseCommunicator_->sendData("CMD2300\n");
}

int Main::sendImu4(bool operational)
{
  if (operational) {
  return baseCommunicator_->sendData("CMD2401\n");
  }

  return baseCommunicator_->sendData("CMD2400\n");
}

int Main::sendImu5(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD2501\n");
  }

  return baseCommunicator_->sendData("CMD2500\n");
}

int Main::sendImu6(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD2601\n");
  }

  return baseCommunicator_->sendData("CMD2600\n");
}

int Main::sendImu7(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD2701\n");
  }

  return baseCommunicator_->sendData("CMD2700\n");
}

int Main::sendImu8(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD2801\n");
  }

  return baseCommunicator_->sendData("CMD2800\n");
}

int Main::sendProxiFront1(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD2901\n");
  }

  return baseCommunicator_->sendData("CMD2900\n");
}

int Main::sendProxiFront2(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD3001\n");
  }

  return baseCommunicator_->sendData("CMD3000\n");
}

int Main::sendProxiFront3(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD3101\n");
  }

  return baseCommunicator_->sendData("CMD3100\n");
}

int Main::sendProxiFront4(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD3201\n");
  }

  return baseCommunicator_->sendData("CMD3200\n");
}

int Main::sendProxiFront5(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD3301\n");
  }

  return baseCommunicator_->sendData("CMD3300\n");
}

int Main::sendProxiFront6(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD3401\n");
  }

  return baseCommunicator_->sendData("CMD3400\n");
}

int Main::sendProxiFront7(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD3501\n");
  }

  return baseCommunicator_->sendData("CMD3500\n");
}

int Main::sendProxiFront8(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD3601\n");
  }

  return baseCommunicator_->sendData("CMD3600\n");
}

int Main::sendProxiRear1(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD3701\n");
  }

  return baseCommunicator_->sendData("CMD3700\n");
}

int Main::sendProxiRear2(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD3801\n");
  }

  return baseCommunicator_->sendData("CMD3800\n");
}

int Main::sendProxiRear3(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD3901\n");
  }

  return baseCommunicator_->sendData("CMD3900\n");
}

int Main::sendProxiRear4(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD4001\n");
  }

  return baseCommunicator_->sendData("CMD4000\n");
}

int Main::sendProxiRear5(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD4101\n");
  }

  return baseCommunicator_->sendData("CMD4100\n");
}

int Main::sendProxiRear6(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD4201\n");
  }

  return baseCommunicator_->sendData("CMD4200\n");
}

int Main::sendProxiRear7(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD4301\n");
  }

  return baseCommunicator_->sendData("CMD4300\n");
}

int Main::sendProxiRear8(bool operational)
{
  if (operational) {
    return baseCommunicator_->sendData("CMD4401\n");
  }

  return baseCommunicator_->sendData("CMD4400\n");
}

void Main::run()
{
  cmn_data_ = data_.getCommunicationsData();
  cmn_data_.run_length = 1250;
  cmn_data_.module_status = data::ModuleStatus::kStart;
  data_.setCommunicationsData(cmn_data_);
  ReceiverThread* receiverThread = new ReceiverThread(baseCommunicator_);
  receiverThread->start();

  while (1) {
    nav_ = data_.getNavigationData();
    mtr_ = data_.getMotorData();
    sen_ = data_.getSensorsData();
    stm_ = data_.getStateMachineData();
    bat_ = data_.getBatteriesData();
    sendDistance(nav_.distance);
    sendVelocity(nav_.velocity);
    sendAcceleration(nav_.acceleration);
    sendRpmFl(mtr_.velocity_1);
    sendRpmFr(mtr_.velocity_2);
    sendRpmBl(mtr_.velocity_3);
    sendRpmBr(mtr_.velocity_4);
    sendState(stm_.current_state);
    sendHpVoltage(bat_.high_power_batteries.at(0));
    sendHpTemperature(bat_.high_power_batteries.at(0));
    sendHpCharge(bat_.high_power_batteries.at(0));
    sendHpVoltage1(bat_.high_power_batteries.at(1));
    sendHpTemperature1(bat_.high_power_batteries.at(1));
    sendHpCharge1(bat_.high_power_batteries.at(1));
    sendLpCharge(bat_.low_power_batteries.at(0));
    sendLpCharge1(bat_.low_power_batteries.at(1));
    sendTorqueFr(mtr_.torque_2);
    sendTorqueFl(mtr_.torque_1);
    sendTorqueBr(mtr_.torque_4);
    sendTorqueBl(mtr_.torque_3);
    sendImu1(sen_.imu.value[0].operational);
    sendImu2(sen_.imu.value[1].operational);
    sendImu3(sen_.imu.value[2].operational);
    sendImu4(sen_.imu.value[3].operational);
    sendImu5(sen_.imu.value[4].operational);
    sendImu6(sen_.imu.value[5].operational);
    sendImu7(sen_.imu.value[6].operational);
    sendImu8(sen_.imu.value[7].operational);
    sendProxiFront1(sen_.proxi_front.value[0].operational);
    sendProxiFront2(sen_.proxi_front.value[1].operational);
    sendProxiFront3(sen_.proxi_front.value[2].operational);
    sendProxiFront4(sen_.proxi_front.value[3].operational);
    sendProxiFront5(sen_.proxi_front.value[4].operational);
    sendProxiFront6(sen_.proxi_front.value[5].operational);
    sendProxiFront7(sen_.proxi_front.value[6].operational);
    sendProxiFront8(sen_.proxi_front.value[7].operational);
    sendProxiRear1(sen_.proxi_back.value[0].operational);
    sendProxiRear2(sen_.proxi_back.value[1].operational);
    sendProxiRear3(sen_.proxi_back.value[2].operational);
    sendProxiRear4(sen_.proxi_back.value[3].operational);
    sendProxiRear5(sen_.proxi_back.value[4].operational);
    sendProxiRear6(sen_.proxi_back.value[5].operational);
    sendProxiRear7(sen_.proxi_back.value[6].operational);
    sendProxiRear8(sen_.proxi_back.value[7].operational);
  }

  receiverThread->join();
  delete receiverThread;
}

}}

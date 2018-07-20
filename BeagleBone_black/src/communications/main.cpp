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

#include <string>
#include <sstream>

namespace hyped {

using utils::System;

namespace communications {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      sys_(System::getSystem()),
      log_(log),
      data_(data::Data::getInstance())
{
  const char* ipAddress = "127.0.0.1"; /*SpaceX will give an IP 192.168.0.6-24*/
  int port_no = 5695;
  base_communicator_ = new Communications(log, ipAddress, port_no);
}

int Main::sendDistance(NavigationType distance)
{
  return base_communicator_->sendData("CMD01" + std::to_string(distance) + "\n");
}

int Main::sendVelocity(NavigationType speed)
{
  return base_communicator_->sendData("CMD02" + std::to_string(speed) + "\n");
}

int Main::sendAcceleration(NavigationType accel)
{
  return base_communicator_->sendData("CMD03" + std::to_string(accel) + "\n");
}

int Main::sendRpmFl(float rpm_fl)
{
  return base_communicator_->sendData("CMD04" + std::to_string(rpm_fl) + "\n");
}

int Main::sendRpmFr(float rpm_fr)
{
  return base_communicator_->sendData("CMD05" + std::to_string(rpm_fr) + "\n");
}

int Main::sendRpmBl(float rpm_bl)
{
  return base_communicator_->sendData("CMD06" + std::to_string(rpm_bl) + "\n");
}

int Main::sendRpmBr(float rpm_br)
{
  return base_communicator_->sendData("CMD07" + std::to_string(rpm_br) + "\n");
}

int Main::sendState(State state)
{
  switch (state) {
    case data::kIdle             : state_code_ = 0; break;
    case data::kCalibrating      : state_code_ = 1; break;
    case data::kReady            : state_code_ = 2; break;
    case data::kAccelerating     : state_code_ = 3; break;
    case data::kDecelerating     : state_code_ = 4; break;
    case data::kEmergencyBraking : state_code_ = 5; break;
    case data::kRunComplete      : state_code_ = 6; break;
    case data::kFailureStopped   : state_code_ = 7; break;
    case data::kExiting          : state_code_ = 8; break;
    case data::kFinished         : state_code_ = 9; break;
    default: break;
  }

  return base_communicator_->sendData("CMD08" + std::to_string(state_code_) + "\n");
}

int Main::sendHpVoltage(Battery hp_battery)
{
  // Convert voltage reading units from decivolts to volts
  return base_communicator_->sendData("CMD09" + std::to_string(hp_battery.voltage / 10.0) + "\n");
}

int Main::sendHpTemperature(Battery hp_battery)
{
  return base_communicator_->sendData("CMD10" + std::to_string(hp_battery.temperature) + "\n");
}

int Main::sendHpCharge(Battery hp_battery)
{
  return base_communicator_->sendData("CMD11" + std::to_string(hp_battery.charge) + "\n");
}

int Main::sendHpVoltage1(Battery hp_battery1)
{
  // Convert voltage reading units from decivolts to volts
  return base_communicator_->sendData("CMD12" + std::to_string(hp_battery1.voltage / 10.0)
                                      + "\n");
}

int Main::sendHpTemperature1(Battery hp_battery1)
{
  return base_communicator_->sendData("CMD13" + std::to_string(hp_battery1.temperature) + "\n");
}

int Main::sendHpCharge1(Battery hp_battery1)
{
  return base_communicator_->sendData("CMD14" + std::to_string(hp_battery1.charge) + "\n");
}

int Main::sendLpCharge(Battery lp_battery)
{
  return base_communicator_->sendData("CMD15" + std::to_string(lp_battery.charge) + "\n");
}

int Main::sendLpCharge1(Battery lp_battery1)
{
  return base_communicator_->sendData("CMD16" + std::to_string(lp_battery1.charge) + "\n");
}

int Main::sendImu(std::array<Imu, Sensors::kNumImus> imus)
{
  std::string sen, sen1, sen2, sen3;
  sen = imus[0].operational ? "1" : "2";
  sen1 = imus[1].operational ? "1" : "2";
  sen2 = imus[2].operational ? "1" : "2";
  sen3 = imus[3].operational ? "1" : "2";

  return base_communicator_->sendData("CMD17" + sen + sen1 + sen2 + sen3 + "\n");
}

int Main::sendProxiFront(std::array<Proximity, Sensors::kNumProximities> proxies_front)
{
  std::string sen, sen1, sen2, sen3, sen4, sen5, sen6, sen7;
  sen = proxies_front[0].operational ? "1" : "2";
  sen1 = proxies_front[1].operational ? "1" : "2";
  sen2 = proxies_front[2].operational ? "1" : "2";
  sen3 = proxies_front[3].operational ? "1" : "2";
  sen4 = proxies_front[4].operational ? "1" : "2";
  sen5 = proxies_front[5].operational ? "1" : "2";
  sen6 = proxies_front[6].operational ? "1" : "2";
  sen7 = proxies_front[7].operational ? "1" : "2";

  return base_communicator_->sendData("CMD18" + sen + sen1 + sen2 + sen3 +
                                      sen4 + sen5 + sen6 + sen7 + "\n");
}

int Main::sendProxiRear(std::array<Proximity, Sensors::kNumProximities> proxies_rear)
{
  std::string sen, sen1, sen2, sen3, sen4, sen5, sen6, sen7;
  sen = proxies_rear[0].operational ? "1" : "2";
  sen1 = proxies_rear[1].operational ? "1" : "2";
  sen2 = proxies_rear[2].operational ? "1" : "2";
  sen3 = proxies_rear[3].operational ? "1" : "2";
  sen4 = proxies_rear[4].operational ? "1" : "2";
  sen5 = proxies_rear[5].operational ? "1" : "2";
  sen6 = proxies_rear[6].operational ? "1" : "2";
  sen7 = proxies_rear[7].operational ? "1" : "2";

  return base_communicator_->sendData("CMD19" + sen + sen1 + sen2 + sen3 +
                                      sen4 + sen5 + sen6 + sen7 + "\n");
}

int Main::sendEmBrakes(bool front_brakes, bool rear_brakes)
{
  std::string brake, brake1;
  brake = front_brakes ? "1" : "2";
  brake1 = rear_brakes ? "1" : "2";

  return base_communicator_->sendData("CMD20" + brake + brake1 + "\n");
}

int Main::sendHpCurrent(Battery hp_battery)
{
  return base_communicator_->sendData("CMD21" + std::to_string(hp_battery.current /10) + "\n");
}

int Main::sendHpCurrent1(Battery hp_battery1)
{
  return base_communicator_->sendData("CMD22" + std::to_string(hp_battery1.current /10) + "\n");
}

int Main::sendLpCurrent(Battery lp_battery)
{
  return base_communicator_->sendData("CMD23" + std::to_string(lp_battery.current /10) + "\n");
}

int Main::sendLpCurrent1(Battery lp_battery1)
{
  return base_communicator_->sendData("CMD24" + std::to_string(lp_battery1.current/ 10) + "\n");
}


void Main::run()
{
  cmn_.launch_command = false;
  cmn_.reset_command = false;
  cmn_.service_propulsion_go = false;
  cmn_.run_length = 1250;

  if (base_communicator_->isConnected()) {
    cmn_.module_status = data::ModuleStatus::kInit;
    data_.setCommunicationsData(cmn_);
  } else {
    cmn_.module_status = data::ModuleStatus::kCriticalFailure;
    data_.setCommunicationsData(cmn_);

    return;  // If connection fails, stops the communication module
  }

  ReceiverThread* receiverThread = new ReceiverThread(log_, base_communicator_);
  receiverThread->start();

  while (sys_.running_) {
    stm_ = data_.getStateMachineData();
    nav_ = data_.getNavigationData();
    mtr_ = data_.getMotorData();
    sen_ = data_.getSensorsData();
    bat_ = data_.getBatteriesData();
    emb_ = data_.getEmergencyBrakesData();

    sendState(stm_.current_state);

    if (nav_.module_status != data::ModuleStatus::kStart) {
      log_.DBG3("COMN", "Send navigation data.");
      sendDistance(nav_.distance);
      sendVelocity(nav_.velocity);
      sendAcceleration(nav_.acceleration);
    }

    if (mtr_.module_status != data::ModuleStatus::kStart) {
      log_.DBG3("COMN", "Send motors data.");
      sendRpmFl(mtr_.velocity_1);
      sendRpmFr(mtr_.velocity_2);
      sendRpmBl(mtr_.velocity_3);
      sendRpmBr(mtr_.velocity_4);
    }

    if (sen_.module_status != data::ModuleStatus::kStart) {
      log_.DBG3("COMN", "Send sensors data.");
      sendImu(sen_.imu.value);
      sendProxiFront(sen_.proxi_front.value);
      sendProxiRear(sen_.proxi_back.value);
    }

    if (bat_.module_status != data::ModuleStatus::kStart) {
      log_.DBG3("COMN", "Send batteries data.");
      sendHpVoltage(bat_.high_power_batteries.at(0));
      sendHpTemperature(bat_.high_power_batteries.at(0));
      sendHpCharge(bat_.high_power_batteries.at(0));
      sendHpVoltage1(bat_.high_power_batteries.at(1));
      sendHpTemperature1(bat_.high_power_batteries.at(1));
      sendHpCharge1(bat_.high_power_batteries.at(1));
      sendLpCharge(bat_.low_power_batteries.at(0));
      sendLpCharge1(bat_.low_power_batteries.at(1));
      sendHpCurrent(bat_.high_power_batteries.at(0));
      sendHpCurrent1(bat_.high_power_batteries.at(1));
      sendLpCurrent(bat_.low_power_batteries.at(0));
      sendLpCurrent1(bat_.low_power_batteries.at(1));
    }

    if (emb_.module_status != data::ModuleStatus::kStart) {
      log_.DBG3("COMN", "Send emergency brakes data.");
      sendEmBrakes(emb_.front_brakes, emb_.rear_brakes);
    }

    sleep(200);
  }
}
}}

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

#ifndef BEAGLEBONE_BLACK_COMMUNICATIONS_MAIN_HPP_
#define BEAGLEBONE_BLACK_COMMUNICATIONS_MAIN_HPP_

#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "utils/system.hpp"
#include "communications/communications.hpp"
#include "communications/receiver.hpp"

namespace hyped {

using data::Sensors;
using data::Imu;
using data::Proximity;
using data::State;
using data::Battery;
using utils::concurrent::Thread;
using utils::Logger;

namespace communications {

class Main : public Thread {
 public:
  explicit Main(uint8_t id, Logger& log);
  void run() override;
  int sendDistance(NavigationType distance);                                          // CMD01
  int sendVelocity(NavigationType speed);                                             // CMD02
  int sendAcceleration(NavigationType accel);                                         // CMD03
  int sendRpmFl(float rpm_fl);                                                        // CMD04
  int sendRpmFr(float rpm_fr);                                                        // CMD05
  int sendRpmBl(float rpm_bl);                                                        // CMD06
  int sendRpmBr(float rpm_br);                                                        // CMD07
  int sendState(State state);                                                         // CMD08
  int sendHpVoltage(Battery hp_battery);                                              // CMD09
  int sendHpTemperature(Battery hp_battery);                                          // CMD10
  int sendHpCharge(Battery hp_battery);                                               // CMD11
  int sendHpVoltage1(Battery hp_battery1);                                            // CMD12
  int sendHpTemperature1(Battery hp_battery1);                                        // CMD13
  int sendHpCharge1(Battery hp_battery1);                                             // CMD14
  int sendLpCharge(Battery lp_battery);                                               // CMD15
  int sendLpCharge1(Battery lp_battery1);                                             // CMD16
  int sendImu(std::array<Imu, Sensors::kNumImus> imus);                               // CMD17
  int sendProxiFront(std::array<Proximity, Sensors::kNumProximities> proxies_front);  // CMD18
  int sendProxiRear(std::array<Proximity, Sensors::kNumProximities> proxies_rear);    // CMD19
  int sendEmBrakes(bool front_brakes, bool rear_brakes);                              // CMD20
  int sendHpCurrent(Battery hp_battery);                                              // CMD21
  int sendHpCurrent1(Battery hp_battery1);                                            // CMD22
  int sendHpLowVoltageCell(Battery hp_battery);                                       // CMD23
  int sendHpHighVoltageCell(Battery hp_battery);                                      // CMD24
  int sendHpLowVoltageCell1(Battery hp_battery1);                                     // CMD25
  int sendHpHighVoltageCell1(Battery hp_battery1);                                    // CMD26
  int sendLpVoltage(Battery lp_battery);                                              // CMD27
  int sendLpVoltage1(Battery lp_battery1);                                            // CMD28
  int sendLpCurrent(Battery lp_battery);                                              // CMD29
  int sendLpCurrent1(Battery lp_battery1);                                            // CMD30


 private:
  int state_code_;
  Communications* base_communicator_;
  utils::System& sys_;
  Logger&        log_;
  data::Data&    data_;
  data::StateMachine stm_;
  data::Motors mtr_;
  data::Sensors sen_;
  data::Batteries bat_;
  data::Navigation nav_;
  data::Communications cmn_;
  data::EmergencyBrakes emb_;
};

}}  //  namespace hyped::communications

#endif  // BEAGLEBONE_BLACK_COMMUNICATIONS_MAIN_HPP_

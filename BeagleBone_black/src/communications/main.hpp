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
#ifdef PROXI
using data::Proximity;
#endif
using data::State;
using data::Battery;
using utils::concurrent::Thread;
using utils::Logger;

namespace communications {

class Main : public Thread {
 public:
  typedef std::array<Imu, Sensors::kNumImus>              ImuArray;
#ifdef PROXI
  typedef std::array<Proximity, Sensors::kNumProximities> ProximityArray;
#endif
  explicit Main(uint8_t id, Logger& log);
  void run() override;
  int sendDistance(NavigationType distance);                // CMD01
  int sendVelocity(NavigationType speed);                   // CMD02
  int sendAcceleration(NavigationType accel);               // CMD03
  int sendRpmFl(float rpm_fl);                              // CMD04
  int sendRpmFr(float rpm_fr);                              // CMD05
  int sendRpmBl(float rpm_bl);                              // CMD06
  int sendRpmBr(float rpm_br);                              // CMD07
  int sendState(State state);                               // CMD08
  int sendImu(ImuArray imus);                               // CMD09
#ifdef PROXI
  int sendProxiFront(ProximityArray proxies_front);         // CMD10
  int sendProxiRear(ProximityArray proxies_rear);           // CMD11
#endif
  int sendEmBrakes(bool front_brakes, bool rear_brakes);    // CMD12
  int sendHpBattery(Battery hpb);                           // CMD13-18
  int sendHpBattery_1(Battery hpb_1);                       // CMD19-24
  int sendLpBattery(Battery lpb);                           // CMD25-27
  int sendLpBattery_1(Battery lpb_1);                       // CMD28-30


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

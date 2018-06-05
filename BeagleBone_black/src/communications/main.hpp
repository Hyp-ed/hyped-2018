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
#include "communications/communications.hpp"
#include "communications/receiver.hpp"

namespace hyped {

using data::State;
using data::Battery;
using utils::concurrent::Thread;
using utils::Logger;

namespace communications {

class Main : public Thread {
 public:
  explicit Main(uint8_t id, Logger& log);
  void run() override;
  int sendDistance(NavigationType distance);    // CMD01
  int sendVelocity(NavigationType speed);       // CMD02
  int sendAcceleration(NavigationType accel);   // CMD03
  int sendStripeCount(int stripes);             // CMD04
  int sendRpmFl(float rpmfl);                   // CMD05
  int sendRpmFr(float rpmfr);                   // CMD06
  int sendRpmBl(float rpmBl);                   // CMD07
  int sendRpmBr(float rpmBr);                   // CMD08
  int sendState(State state);                   // CMD09
  int sendHpVoltage(Battery hpBattery);        // CMD10
  int sendHpTemperature(Battery hpBattery);    // CMD11
  int sendHpVoltage1(Battery hpBattery1);       // CMD12
  int sendHpTemperature1(Battery hpBattery1);   // CMD13

 private:
  int stateCode_;
  Communications* baseCommunicator_;
  data::Data& data_ = data::Data::getInstance();
  data::Navigation nav_;
  data::Motors mtr_;
  data::Sensors sns_;
  data::StateMachine stm_;
  data::Batteries bat_;
};

}}  //  namespace hyped::communications

#endif  // BEAGLEBONE_BLACK_COMMUNICATIONS_MAIN_HPP_

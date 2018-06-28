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
  int sendRpmFl(float rpmfl);                   // CMD04
  int sendRpmFr(float rpmfr);                   // CMD05
  int sendRpmBl(float rpmBl);                   // CMD06
  int sendRpmBr(float rpmBr);                   // CMD07
  int sendState(State state);                   // CMD08
  int sendHpVoltage(Battery hpBattery);         // CMD09
  int sendHpTemperature(Battery hpBattery);     // CMD10
  int sendHpCharge(Battery hpBattery);          // CMD11
  int sendHpVoltage1(Battery hpBattery1);       // CMD12
  int sendHpTemperature1(Battery hpBattery1);   // CMD13
  int sendHpCharge1(Battery hpBattery1);        // CMD14
  int sendLpCharge(Battery lpBattery);          // CMD15
  int sendLpCharge1(Battery lpBattery1);        // CMD16
  int sendTorqueFl(float torquefl);             // CMD17
  int sendTorqueFr(float torquefr);             // CMD18
  int sendTorqueBl(float torquebl);             // CMD19
  int sendTorqueBr(float torquebr);             // CMD20
  int sendImu1(bool operational);               // CMD21
  int sendImu2(bool operational);               // CMD22
  int sendImu3(bool operational);               // CMD23
  int sendImu4(bool operational);               // CMD24
  int sendImu5(bool operational);               // CMD25
  int sendImu6(bool operational);               // CMD26
  int sendImu7(bool operational);               // CMD27
  int sendImu8(bool operational);               // CMD28
  int sendProxiFront1(bool operational);        // CMD29
  int sendProxiFront2(bool operational);        // CMD30
  int sendProxiFront3(bool operational);        // CMD31
  int sendProxiFront4(bool operational);        // CMD32
  int sendProxiFront5(bool operational);        // CMD33
  int sendProxiFront6(bool operational);        // CMD34
  int sendProxiFront7(bool operational);        // CMD35
  int sendProxiFront8(bool operational);        // CMD36
  int sendProxiRear1(bool operational);         // CMD37
  int sendProxiRear2(bool operational);         // CMD38
  int sendProxiRear3(bool operational);         // CMD39
  int sendProxiRear4(bool operational);         // CMD40
  int sendProxiRear5(bool operational);         // CMD41
  int sendProxiRear6(bool operational);         // CMD42
  int sendProxiRear7(bool operational);         // CMD43
  int sendProxiRear8(bool operational);         // CMD44

 private:
  int stateCode_;
  Communications* baseCommunicator_;
  data::Data& data_ = data::Data::getInstance();
  data::Navigation nav_;
  data::Motors mtr_;
  data::StateMachine stm_;
  data::Batteries bat_;
  data::Communications cmn_data_;
  data::Sensors sen_;
};

}}  //  namespace hyped::communications

#endif  // BEAGLEBONE_BLACK_COMMUNICATIONS_MAIN_HPP_

/*
 * Authors: Kofi and Isabella
 * Organisation: HYPED
 * Date: 15/04/18
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

#include "communications/receiver.hpp"

namespace hyped {

using utils::System;

namespace communications {

ReceiverThread::ReceiverThread(Logger& log, Communications* base_communicator)
    : Thread(),
      sys_(System::getSystem()),
      log_(log),
      data_(data::Data::getInstance()),
      base_communicator_(base_communicator)
{ /* Empty */ }

void ReceiverThread::run()
{
  data::Communications cmn_;

  while (sys_.running_) {
    cmn_ = data_.getCommunicationsData();
    log_.DBG1("COMN", "Before receiving message (launch=%d, reset=%d, length=%.3fm, spg=%d)",
        cmn_.launch_command, cmn_.reset_command, cmn_.run_length, cmn_.service_propulsion_go);
    int command = base_communicator_->receiveMessage();

    switch (command) {
      case 0:
        log_.INFO("COMN", "Received 0 (ACK FROM SERVER)");          // 0: ACK
        break;
      case 1:
        cmn_.module_status = data::ModuleStatus::kCriticalFailure;
        log_.INFO("COMN", "Received 1 (STOP)");                     // 1: STOP
        break;
      case 2:
        cmn_.launch_command = true;
        log_.INFO("COMN", "Received 2 (LAUNCH)");                   // 2: LAUNCH
        break;
      case 3:
        cmn_.reset_command = true;
        log_.INFO("COMN", "Received 3 (RESET)");                    // 3: RESET
        break;
      case 4:
        cmn_.run_length = static_cast<float>(base_communicator_->receiveRunLength());
        log_.INFO("COMN", "Received 4 (TRACK LENGTH = %.3fm)", cmn_.run_length);  // 4: TRACK LENGTH
        break;
      case 5:
        cmn_.service_propulsion_go = true;
        log_.INFO("COMN", "Received 5 (SERVICE PROPULSION GO)");    // 5: SERVICE PROPULSION GO
        break;
      case 6:
        cmn_.service_propulsion_go = false;
        log_.INFO("COMN", "Received 6 (SERVICE PROPULSION STOP)");  // 6: SERVICE PROPULSION STOP
        break;
      default:
        cmn_.module_status = data::ModuleStatus::kCriticalFailure;  // DEFAULT: Critical Failure
        data_.setCommunicationsData(cmn_);
        log_.ERR("COMN", "CONNECTION LOST (STOP)");
        return;
    }

    data_.setCommunicationsData(cmn_);
    log_.DBG1("COMN", "After receiving message (launch=%d, reset=%d, length=%.3fm, spg=%d)",
        cmn_.launch_command, cmn_.reset_command, cmn_.run_length, cmn_.service_propulsion_go);
  }
}


}}

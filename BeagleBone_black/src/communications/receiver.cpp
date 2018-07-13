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
namespace communications {

ReceiverThread::ReceiverThread(Logger& log, Communications* base_communicator)
    : Thread(),
      log_(log),
      base_communicator_(base_communicator),
      data_(data::Data::getInstance())
{ /* Empty */ }

void ReceiverThread::run()
{
  data::Communications cmn_data;

  while (1) {
    cmn_data = data_.getCommunicationsData();
    int command = base_communicator_->receiveMessage();

    switch (command) {
      case 0:
        log_.INFO("COMN", "Received 0 (ACK FROM SERVER)");
        break;
      case 1:
        log_.INFO("COMN", "Received 1 (STOP)");  // STOP
        cmn_data.module_status = data::ModuleStatus::kCriticalFailure;
        break;
      case 2:
        log_.INFO("COMN", "Received 2 (LAUNCH)");  // LAUNCH
        cmn_data.launch_command = true;
        break;
      case 3:
        log_.INFO("COMN", "Received 3 (RESET)");  // RESET
        cmn_data.reset_command = true;
        break;
      case 4:
        log_.INFO("COMN", "Received 4 (TRACK LENGTH)");  // TRACK LENGTH
        cmn_data.run_length = static_cast<float>(base_communicator_->receiveRunLength())/1000;
        break;
      case 5:
        log_.INFO("COMN", "Received 5 (SERVICE PROPULSION GO)");  // SERVICE PROPULSION GO
        cmn_data.service_propulsion_go = true;
        break;
      case 6:
        log_.INFO("COMN", "Received 6 (SERVICE PROPULSION STOP)");  // SERVICE PROPULSION STOP
        cmn_data.service_propulsion_go = false;
        break;
      default:
        log_.ERR("COMN", "Received %d (Should not reach here)", command);
        break;
    }

    data_.setCommunicationsData(cmn_data);
  }
}

}}

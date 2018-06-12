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

ReceiverThread::ReceiverThread(Communications* baseCommunicator)
    : Thread(),
      baseCommunicator_(baseCommunicator),
      data_(data::Data::getInstance())
{ /* Empty */ }

void ReceiverThread::run()
{
  data::Communications cmn_data;

  while (1) {
    int command = baseCommunicator_->receiveMessage();

    switch (command) {
      case 1:
        cmn_data.stopCommand = true;
        break;
      case 2:
        cmn_data.launchCommand = true;
        break;
      case 3:
        cmn_data.resetCommand = true;
        break;
      case 4:
        cmn_data.run_length = static_cast<float>(baseCommunicator_->receiveRunLength())/1000;
        break;
    }

    data_.setCommunicationsData(cmn_data);
  }
}

}}

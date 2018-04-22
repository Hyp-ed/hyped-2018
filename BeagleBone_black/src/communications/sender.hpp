/*
 * Author: Kofi and Isabella
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

#ifndef BEAGLEBONE_BLACK_COMMUNICATIONS_SENDER_HPP_
#define BEAGLEBONE_BLACK_COMMUNICATIONS_SENDER_HPP_

#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "communications/communications.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;

namespace communications {

class SenderThread : public Thread {
 public:
  explicit SenderThread(Communications* baseCommunicator);
  void run() override;

 private:
  Communications* baseCommunicator;
  data::Data& data = data::Data::getInstance();
  data::Navigation nav;
  data::Motors mtr;
};

}}  //  namespace hyped::communications

#endif  // BEAGLEBONE_BLACK_COMMUNICATIONS_SENDER_HPP_
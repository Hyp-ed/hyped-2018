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
#include "communications/sender.hpp"
#include "communications/receiver.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;

namespace communications {

class Main : public Thread {
 public:
  explicit Main(uint8_t id, Logger& log);
  void run() override;

 private:
  Communications* baseCommunicator_;
  data::Data& data_ = data::Data::getInstance();
  data::Navigation nav_;
  data::Motors mtr_;
};

}}  //  namespace hyped::communications

#endif  // BEAGLEBONE_BLACK_COMMUNICATIONS_MAIN_HPP_

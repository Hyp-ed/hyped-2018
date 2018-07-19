/*
 * Author: Brano, Uday, Adi
 * Organisation: HYPED
 * Date: 18 March 2018
 * Description: Main file for navigation class.
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *    except in compliance with the License. You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software distributed under
 *    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *    either express or implied. See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef BEAGLEBONE_BLACK_NAVIGATION_MAIN_HPP_
#define BEAGLEBONE_BLACK_NAVIGATION_MAIN_HPP_

#include <cstdint>

#include "data/data.hpp"
#include "navigation/navigation.hpp"
#include "utils/concurrent/lock.hpp"
#include "utils/concurrent/thread.hpp"

namespace hyped {

using data::Sensors;
using utils::concurrent::Lock;
using utils::concurrent::Thread;
using utils::Logger;

namespace navigation {

class Main: public Thread {
 public:
  Main(uint8_t id, Logger& log);
  void run() override;
  const Navigation::FullOutput& getAllNavData();

 private:
  bool imuChanged(const Sensors& old_data, const Sensors& new_data);
  bool proxiChanged(const Sensors& old_data, const Sensors& new_data);
  inline bool stripeCntChanged(const Sensors& old_data, const Sensors& new_data);
  inline bool opticalEncDistChanged(const Sensors& old_data, const Sensors& new_data);
  void updateData();


  data::Data& data_;
  Navigation nav_;
  Lock l_;
};

}}  // namespace hyped::navigation

#endif  // BEAGLEBONE_BLACK_NAVIGATION_MAIN_HPP_

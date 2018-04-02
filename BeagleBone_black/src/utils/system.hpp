/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 2. April 2018
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

#ifndef BEAGLEBONE_BLACK_UTILS_SYSTEM_HPP_
#define BEAGLEBONE_BLACK_UTILS_SYSTEM_HPP_

#include <cstdint>

namespace hyped {
namespace utils {

class System {
 public:
  static void parseArgs(int argc, char* argv[]);
  static System& getSystem();

  // runtime arguments to configure the whole system
  bool verbose;
  bool verbose_motor;
  bool verbose_nav;
  bool verbose_sensor;
  bool verbose_state;

  int8_t debug;
  int8_t debug_motor;
  int8_t debug_nav;
  int8_t debug_sensor;
  int8_t debug_state;

 private:
  System() = delete;
  System(int argc, char* argv[]);
  static System* system_;
};

}}  // namespace hyped::utils

#endif  // BEAGLEBONE_BLACK_UTILS_SYSTEM_HPP_

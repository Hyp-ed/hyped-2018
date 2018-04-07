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

#ifndef BEAGLEBONE_BLACK_UTILS_LOGGER_HPP_
#define BEAGLEBONE_BLACK_UTILS_LOGGER_HPP_

#include <cstdint>

namespace hyped {
namespace utils {

class Logger {
 public:
  explicit Logger(bool verbose, int8_t debug);

  void INFO(const char *format, ...);

  void DBG(const char *format, ...);
  void DBG0(const char *format, ...);
  void DBG1(const char *format, ...);
  void DBG2(const char *format, ...);
  void DBG3(const char *format, ...);

 private:
  bool verbose_;
  int8_t debug_;
};

}}  // namespace hyped::utils
#endif  // BEAGLEBONE_BLACK_UTILS_LOGGER_HPP_

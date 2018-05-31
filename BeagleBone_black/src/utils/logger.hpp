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
  Logger()
      : verbose_(false),
        debug_(-1)
  { /* EMPTY */ }

  /**
   * @brief Construct a new Logger object with preset debug levels
   *
   * @param verbose
   * @param debug   - debug level used in DBGX()
   */
  explicit Logger(bool verbose, int8_t debug);

  /**
   * @brief All debug messages have the same format. The arguments closely
   * follow the format-string signature of printf function.
   *
   * INFO is printed iff verbose_ for this logger set.
   * Output goes to STDOUT
   *
   * @param module - name of the module generating the message
   * @param format - format string as in printf
   * @param ...    - arguments for the format string
   */
  void INFO(const char* module, const char* format, ...);

  /**
   * @brief Use to indicate error. Output goes to STDERR
   */
  void ERR(const char* module, const char* format, ...);

  /**
   * @brief Use for infrequent debug messages:
   * e.g. state transitions, successful initialisation
   * DBG is printed iff debug_ >= 0
   */
  void DBG(const char* module, const char* format, ...);

  /**
   * @brief Use for medium frequency debug messages
   * e.g. successful sensor reading, receiving command message from basestation
   * DBG is printed iff debug_ >= 1
   */
  void DBG1(const char* module, const char* format, ...);

  /**
   * @brief Use for high frequency debug messages
   * e.g. CAN readings, Update of NAV data
   * DBG is printed iff debug_ >= 2
   */
  void DBG2(const char* module, const char* format, ...);

  /**
   * @brief Use for high frequency debug messages, full debug with detailed output
   * e.g. actual data being received by CAN, each state transtition condition evaluation
   * DBG is printed iff debug_ >= 3
   */
  void DBG3(const char* module, const char* format, ...);

 private:
  bool verbose_;
  int8_t debug_;
};

}}  // namespace hyped::utils
#endif  // BEAGLEBONE_BLACK_UTILS_LOGGER_HPP_

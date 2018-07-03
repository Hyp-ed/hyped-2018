/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 02/07/18
 * Description: Driver for the OPB720B-12Z optical encoder
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


#include <stdio.h>
#include "utils/io/gpio.hpp"
#include "utils/system.hpp"
#include "utils/logger.hpp"

using hyped::utils::io::GPIO;
using hyped::utils::System;
using hyped::utils::Logger;
namespace io = hyped::utils::io;


int main(int argc, char* argv[]) {
  System::parseArgs(argc, argv);
  Logger log(true, 1);

  GPIO the_pin(60, io::gpio::kIn);
  uint8_t val = the_pin.wait();
  int count = 0;
  log.INFO("Opt-En", "Starting optical encoder / keyence");

  while (1) {
      val = the_pin.wait();
      if (val == 1) {
        count++;
      log.INFO("Opt-En", "count: %d", count);
      }
  }
}
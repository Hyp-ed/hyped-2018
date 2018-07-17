/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 18/02/2018
 * Description: This is to show the usage of the fake proximity class
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

#include "sensors/fake_proxi.hpp"

#include <cstdio>
#include <unistd.h>

#include "utils/logger.hpp"
#include "utils/concurrent/thread.hpp"

using hyped::data::Proximity;
using hyped::sensors::FakeProxi;
using hyped::utils::Logger;
using hyped::utils::concurrent::Thread;

int main()
{
  Logger log(1, true);
  Proximity reading;
  FakeProxi generator(log, 23, 1, true);
  FakeProxi file(log, "../BeagleBone_black/data/in/fake_proxi_input.txt");

  log.INFO("Fake-Proxi", "From file....");
  for (int i = 0; i < 3; i++) {
    file.getData(&reading);
    log.INFO("Fake-Proxi", "From file: %d", reading.val);
    Thread::sleep(10);
  }

  log.INFO("Fake-Proxi", "From generator....");
  for (int i = 0; i < 3; i++) {
    generator.getData(&reading);
    log.INFO("Fake-Proxi", "From generator: %d", reading.val);
    Thread::sleep(10);
  }

}


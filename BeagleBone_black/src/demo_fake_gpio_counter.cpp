/*
 * Author: Jack Horsburgh
 * Organisation: HYPED
 * Date: 10/07/2018
 * Description: This is to show the usage of the fake gpio counter class
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

#include "sensors/fake_gpio_counter.hpp"

#include <cstdio>
#include <unistd.h>

#include "state_machine/hyped-machine.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/system.hpp"
#include "utils/logger.hpp"
#include "data/data.hpp"
#include "utils/concurrent/thread.hpp"

using hyped::utils::Logger;
using hyped::sensors::FakeGpioCounter;
using hyped::utils::concurrent::Thread;
using hyped::data::StripeCounter;

int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  Logger log(true, 1);
  StripeCounter stripe_counter;
  FakeGpioCounter fake_gpio_counter(log, "../BeagleBone_black/data/in/fake_keyence_input.txt");

  for (int i=0; i<20; i++) {
    stripe_counter = fake_gpio_counter.getStripeCounter();
    log.INFO("Fake-Gpio-counter", "Stripes seen: %d", stripe_counter.count.value);
    Thread::sleep(50);
  }
}

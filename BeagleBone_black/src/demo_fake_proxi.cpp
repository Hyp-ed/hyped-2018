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

using hyped::data::Proximity;
using hyped::sensors::FakeProxi;

int main()
{
  Proximity reading;
  FakeProxi generator(0, 1);
  FakeProxi file("src/fake_proxi_input.txt");

  for (int i = 0; i < 3; i++) {
    file.getData(&reading);
    printf("From file: %d @ time: null\n", reading.val);

    generator.getData(&reading);
    printf("From generator: %d @ time: null\n", reading.val);

    usleep(10);
  }
}


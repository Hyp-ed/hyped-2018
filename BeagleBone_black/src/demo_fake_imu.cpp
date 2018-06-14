/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 18/02/2018
 * Description: This is to show the usage of the fake imu class
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

#include "sensors/fake_imu.hpp"

#include <cstdio>
#include <unistd.h>

using hyped::data::Imu;
using hyped::data::NavigationVector;
using hyped::sensors::FakeImu;

int main()
{
  Imu reading;
  FakeImu generator(NavigationVector(0), NavigationVector(1), NavigationVector(0), NavigationVector(1));
  FakeImu file("src/fake_imu_input_acc.txt", "src/fake_imu_input_gyr.txt");

  for (int i=0; i<3; i++) {
    file.getData(&reading);
    printf("From file: %fm/s^2 @ %d\n", reading.acc.value[0], reading.acc.timestamp);
    printf("From file: %frad/s @ %d\n", reading.gyr.value[0], reading.gyr.timestamp);

    generator.getData(&reading);
    printf("From generator: %fm/s^2 @ %d\n", reading.acc.value[0], reading.acc.timestamp);
    printf("From generator: %frad/s @ %d\n", reading.gyr.value[0], reading.gyr.timestamp);

    usleep(10);
  }
}

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

#include "state_machine/hyped-machine.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/system.hpp"
#include "utils/logger.hpp"
#include "data/data.hpp"

using hyped::data::Imu;
using hyped::data::NavigationVector;
using hyped::sensors::FakeImu;
using hyped::utils::concurrent::Thread;
using hyped::utils::Logger;

int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  Imu reading;
  Logger log(true, 1);
  FakeImu file(log, "src/fake_imu_input_acc.txt", "src/fake_imu_input_gyr.txt");

  for (int i=0; i<20; i++) {
    file.getData(&reading);
    printf("Accel Readings: x: %f m/s^2, y: %f m/s^2, z: %f m/s^2\n", reading.acc[0], reading.acc[1], reading.acc[2]);
    printf("Gyros Readings: x: %f rad/s, y: %f rad/s, z: %f rad/s\n", reading.gyr[0], reading.gyr[1], reading.gyr[2]);
      
    // TODO(Anyone) change the state of the state machine to accelerating
    Thread::sleep(50);
  }
}

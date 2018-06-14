/*
 * Author: Brano, Adi, Uday
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

#include "navigation/main.hpp"

#include <memory>

#include "utils/system.hpp"

namespace hyped {

using data::Sensors;
using utils::System;

namespace navigation {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      data_(data::Data::getInstance()),
      nav_(System::getSystem().navigation_motors_sync_)
{
  updateData();
}

void Main::run()
{
  std::unique_ptr<Sensors> last_readings(new Sensors());
  std::unique_ptr<Sensors> readings(new Sensors());
  log_.INFO("NAVIGATION", "Main started");

  *last_readings = data_.getSensorsData();  // TODO(Brano): Make sure data_ is properly initd
  while (1) {
    // State updates
    State current_state = data_.getStateMachineData().current_state;
    switch (current_state) {
      case State::kIdle :
        yield();
        continue;
      case State::kCalibrating :
        if (!nav_.is_calibrating_) {
          if (nav_.startCalibration()) {
            log_.INFO("NAV", "Calibration started");
          } else {
            log_.ERR("NAV", "Calibration couldn't start");
            yield();
            continue;
          }
        }
      case State::kReady :
        break;
      case State::kAccelerating :
        if (nav_.is_calibrating_) {
          if (nav_.finishCalibration())
            log_.INFO("NAV", "Calibration finished");
          else
            log_.ERR("NAV", "Calibration couldn't finish");
        }
      default:
        break;
    }

    // Data updates
    *readings = data_.getSensorsData();

    // TODO(Brano): Accelerations and gyros should be in separate arrays in data::Sensors.
    if (!imuChanged(*last_readings, *readings)) {
      // let other threads run, maybe someone will update the sensors data
      yield();
      continue;
    }
    if (proxiChanged(*last_readings, *readings) && stripeCntChanged(*last_readings, *readings))
      nav_.update(readings->imu, readings->proxi, readings->stripe_counter.count);
    else if (proxiChanged(*last_readings, *readings))
      nav_.update(readings->imu, readings->proxi);
    else if (stripeCntChanged(*last_readings, *readings))
      nav_.update(readings->imu, readings->stripe_counter.count);
    else
      nav_.update(readings->imu);

    updateData();

    readings.swap(last_readings);
  }
}

bool Main::imuChanged(const Sensors& old_data, const Sensors& new_data)
{
  for (uint8_t i = 0; i < new_data.imu.size(); ++i) {
    if (new_data.imu[i].gyr.timestamp != old_data.imu[i].gyr.timestamp ||
        new_data.imu[i].acc.timestamp != old_data.imu[i].acc.timestamp)
      return true;
  }
  return false;
}

bool Main::proxiChanged(const Sensors& old_data, const Sensors& new_data)
{
  for (uint8_t i = 0; i < new_data.proxi.size(); ++i) {
    // TODO(Brano): Timestamp proxi data in data::Sensors
    if (new_data.proxi[i].val != old_data.proxi[i].val)
      return true;
  }
  return false;
}

inline bool Main::stripeCntChanged(const Sensors& old_data, const Sensors& new_data)
{
  return new_data.stripe_counter.count.timestamp != old_data.stripe_counter.count.timestamp;
}

void Main::updateData()
{
  data::Navigation nav_data;

  nav_data.module_status              = nav_.getStatus();
  nav_data.distance                   = nav_.getDisplacement();
  nav_data.velocity                   = nav_.getVelocity();
  nav_data.acceleration               = nav_.getAcceleration();
  nav_data.emergency_braking_distance = nav_.getEmergencyBrakingDistance();

  data_.setNavigationData(nav_data);
}

}}  // namespace hyped::navigation

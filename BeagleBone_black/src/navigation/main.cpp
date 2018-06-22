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
using data::State;
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

  // Views of the Sensors structs
  std::unique_ptr<Navigation::ProximityArray> last_proxis(new Navigation::ProximityArray());
  std::unique_ptr<Navigation::ProximityArray> proxis(new Navigation::ProximityArray());
  // Set up pointers as to unify the front and rear proxis in a single array
  for (int i = 0; i < Sensors::kNumProximities; ++i) {
    (*proxis)[i] = &(readings->proxi_front.value[i]);
    (*proxis)[i + Sensors::kNumProximities] = &(readings->proxi_back.value[i]);
    (*last_proxis)[i] = &(last_readings->proxi_front.value[i]);
    (*last_proxis)[i + Sensors::kNumProximities] = &(last_readings->proxi_back.value[i]);
  }
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
      nav_.update(readings->imu, *proxis, readings->stripe_counter.count);
    else if (proxiChanged(*last_readings, *readings))
      nav_.update(readings->imu, *proxis);
    else if (stripeCntChanged(*last_readings, *readings))
      nav_.update(readings->imu, readings->stripe_counter.count);
    else
      nav_.update(readings->imu);

    updateData();

    readings.swap(last_readings);
    proxis.swap(last_proxis);
  }
}

bool Main::imuChanged(const Sensors& old_data, const Sensors& new_data)
{
  return new_data.imu.timestamp != old_data.imu.timestamp;
}

bool Main::proxiChanged(const Sensors& old_data, const Sensors& new_data)
{
  // Both front and back should be always updated at the same time
  return old_data.proxi_front.timestamp != new_data.proxi_front.timestamp &&
         old_data.proxi_back.timestamp  != new_data.proxi_back.timestamp;
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

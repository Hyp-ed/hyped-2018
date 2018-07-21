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

using data::ModuleStatus;
using data::State;
using utils::concurrent::ScopedLock;
using utils::System;

namespace navigation {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      data_(data::Data::getInstance()),
      nav_(System::getSystem().navigation_motors_sync_, log)
{
  updateData();
}

void Main::run()
{
  std::unique_ptr<Sensors> last_readings(new Sensors());
  std::unique_ptr<Sensors> readings(new Sensors());

  // Views of the Sensors structs
#ifdef PROXI
  std::unique_ptr<Navigation::ProximityArray> last_proxis(new Navigation::ProximityArray());
  std::unique_ptr<Navigation::ProximityArray> proxis(new Navigation::ProximityArray());
  // Set up pointers as to unify the front and rear proxis in a single array
  for (int i = 0; i < Sensors::kNumProximities; ++i) {
    (*proxis)[i] = &(readings->proxi_front.value[i]);
    (*proxis)[i + Sensors::kNumProximities] = &(readings->proxi_back.value[i]);
    (*last_proxis)[i] = &(last_readings->proxi_front.value[i]);
    (*last_proxis)[i + Sensors::kNumProximities] = &(last_readings->proxi_back.value[i]);
  }
#endif
  log_.INFO("NAV", "Main started");

  System& sys = System::getSystem();
  while (sys.running_) {
    ScopedLock L(&l_);
    // State updates
    State current_state = data_.getStateMachineData().current_state;
    switch (current_state) {
      case State::kIdle :
        *readings = data_.getSensorsData();
        if ((readings->module_status == ModuleStatus::kInit ||
             readings->module_status == ModuleStatus::kReady) &&
            nav_.getStatus() == ModuleStatus::kStart) {  // NOLINT [whitespace/braces]
          nav_.init(data_.getCalibrationData(), *readings);
          *last_readings = data_.getSensorsData();
          *readings = data_.getSensorsData();
          updateData();
        }
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
        break;
      case State::kReady :
        yield();
        continue;
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
    // check if time goes backwards, ignore such readings
    if (readings->imu.timestamp < last_readings->imu.timestamp) {
      log_.ERR("NAV", "new reading has past timestamp %u", readings->imu.timestamp);
      yield();
      continue;
    }

    // TODO(Brano): Accelerations and gyros should be in separate arrays in data::Sensors.
    if (!imuChanged(*last_readings, *readings)) {
      // let other threads run, maybe someone will update the sensors data
      yield();
      continue;
    }
    Navigation::Input input;
#ifdef PROXI
     if (proxiChanged(*last_readings, *readings)) {
      input.proxis = &(*proxis);
     }
#endif
     if (stripeCntChanged(*last_readings, *readings)) {
      input.sc = &readings->keyence_stripe_counter;
     }
     if (opticalEncDistChanged(*last_readings, *readings)) {
      input.optical_enc_distance = &readings->optical_enc_distance;
     }
     if (imuChanged(*last_readings, *readings)) {
      input.imus = &readings->imu;
     }
     nav_.update(input);

    updateData();

    readings.swap(last_readings);
#ifdef PROXI
    proxis.swap(last_proxis);
#endif
  }
}

const Navigation::FullOutput& Main::getAllNavData()
{
  ScopedLock L(&l_);
  return nav_.getAll();
}

bool Main::imuChanged(const Sensors& old_data, const Sensors& new_data)
{
  return new_data.imu.timestamp != old_data.imu.timestamp;
}

#ifdef PROXI
bool Main::proxiChanged(const Sensors& old_data, const Sensors& new_data)
{
  // Both front and back should be always updated at the same time
  return old_data.proxi_front.timestamp != new_data.proxi_front.timestamp &&
         old_data.proxi_back.timestamp  != new_data.proxi_back.timestamp;
}
#endif

inline bool Main::stripeCntChanged(const Sensors& old_data, const Sensors& new_data)
{
  for (int i = 0; i < Sensors::kNumKeyence; i++) {
    if (new_data.keyence_stripe_counter[i].count.value != old_data.keyence_stripe_counter[i].count.value) //NOLINT
      return true;
  }

  return false;
}

inline bool Main::opticalEncDistChanged(const Sensors& old_data, const Sensors& new_data)
{
  for (int i = 0; i < Sensors::kNumOptEnc; i++) {
    if (new_data.optical_enc_distance[i] != old_data.optical_enc_distance[i])
      return true;
  }

  return false;
}

void Main::updateData()
{
  data::Navigation nav_data;

  nav_data.module_status              = nav_.getStatus();
  nav_data.distance                   = nav_.getDisplacement();
  nav_data.velocity                   = nav_.getVelocity();
  nav_data.acceleration               = nav_.getAcceleration();
  nav_data.emergency_braking_distance = nav_.getEmergencyBrakingDistance();
  nav_data.braking_distance           = nav_.getBrakingDistance();

  data_.setNavigationData(nav_data);
  log_.DBG1("NAV",
      "Update: ms=%d, a=(%.2f, %.2f, %.2f), v=(%.2f, %.2f, %.2f), d=(%.2f, %.2f, %.2f), bd=%.2f, ebd=%.2f, calib=%d", //NOLINT
      nav_.status_,
      nav_.acceleration_[0], nav_.acceleration_[1], nav_.acceleration_[2],
      nav_.velocity_.value[0], nav_.velocity_.value[1], nav_.velocity_.value[2],
      nav_.displacement_.value[0], nav_.displacement_.value[1], nav_.displacement_.value[2],
      nav_.getBrakingDistance(), nav_.getEmergencyBrakingDistance(), nav_.num_gravity_samples_);
}

}}  // namespace hyped::navigation

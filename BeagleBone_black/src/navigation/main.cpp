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

namespace hyped {

using data::Sensors;

namespace navigation {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log),
      data_(data::Data::getInstance()),
      nav_()
{/* EMPTY */}

void Main::run()
{
  data::Navigation nav_data;
  std::unique_ptr<Sensors> last_readings(new Sensors());
  std::unique_ptr<Sensors> readings(new Sensors());

  // Views of the Sensors structs
  std::unique_ptr<Navigation::ProximityArray> last_proxis(new Navigation::ProximityArray());
  std::unique_ptr<Navigation::ProximityArray> proxis(new Navigation::ProximityArray());
  // Set up pointers as to unify the front and rear proxis in a single array
  for (int i = 0; i < Sensors::kNumProximities; ++i) {
    (*proxis)[i] = &(readings->proxi_front[i]);
    (*proxis)[i + Sensors::kNumProximities] = &(readings->proxi_back[i]);
    (*last_proxis)[i] = &(last_readings->proxi_front[i]);
    (*last_proxis)[i + Sensors::kNumProximities] = &(last_readings->proxi_back[i]);
  }
  log_.INFO("NAVIGATION", "Main started");

  *last_readings = data_.getSensorsData();  // TODO(Brano): Make sure data_ is properly initd
  while (1) {
    *readings = data_.getSensorsData();

    // TODO(Brano): Accelerations and gyros should be in separate arrays in data::Sensors.
    if (!imuChanged(*last_readings, *readings)) {
      // let other threads run, maybe someone will update the sensors data
      yield();
      continue;
    }
    if (proxiChanged(*last_proxis, *proxis) && stripeCntChanged(*last_readings, *readings))
      nav_.update(readings->imu, *proxis, readings->stripe_counter.count);
    else if (proxiChanged(*last_proxis, *proxis))
      nav_.update(readings->imu, *proxis);
    else if (stripeCntChanged(*last_readings, *readings))
      nav_.update(readings->imu, readings->stripe_counter.count);
    else
      nav_.update(readings->imu);

    nav_data.distance                   = nav_.getDisplacement();
    nav_data.velocity                   = nav_.getVelocity();
    nav_data.acceleration               = nav_.getAcceleration();
    nav_data.emergency_braking_distance = nav_.getEmergencyBrakingDistance();
    data_.setNavigationData(nav_data);

    readings.swap(last_readings);
    proxis.swap(last_proxis);
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

bool Main::proxiChanged(const Navigation::ProximityArray& old_data,
                        const Navigation::ProximityArray& new_data)
{
  for (uint8_t i = 0; i < new_data.size(); ++i) {
    // TODO(Brano): Timestamp proxi data in data::Sensors
    if (new_data[i]->val != old_data[i]->val)
      return true;
  }
  return false;
}

inline bool Main::stripeCntChanged(const Sensors& old_data, const Sensors& new_data)
{
  return new_data.stripe_counter.count.timestamp != old_data.stripe_counter.count.timestamp;
}

}}  // namespace hyped::navigation

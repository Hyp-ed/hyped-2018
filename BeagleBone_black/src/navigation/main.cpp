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

namespace hyped {

using data::Sensors;

namespace navigation {

Main::Main(uint8_t id, Logger& log)
    : Thread(id, log)
    , data_(data::Data::getInstance())
    , nav_()
{/* EMPTY */}

void Main::run()
{
  data::Navigation nav_data;
  Sensors last_readings = data_.getSensorsData();  // TODO(Brano): Make sure data_ is properly initd
  while (1) {
    Sensors readings = data_.getSensorsData();

    // TODO(Brano): Accelerations and gyros should be in separate arrays in data::Sensors.
    if (!imuChanged(last_readings, readings))
      continue;
    if (proxiChanged(last_readings, readings) && stripeCntChanged(last_readings, readings))
      nav_.update(readings.imu, readings.proxy, readings.stripe_cnt);
    else if (proxiChanged(last_readings, readings))
      nav_.update(readings.imu, readings.proxy);
    else if (stripeCntChanged(last_readings, readings))
      nav_.update(readings.imu, readings.stripe_cnt);
    else
      nav_.update(readings.imu);

    nav_data.distance = nav_.get_displacement();
    nav_data.velocity = nav_.get_velocity();
    nav_data.acceleration = nav_.get_accleration();
    // TODO(Brano): Add stripe count or remove it from data::Navigation.
    data_.setNavigationData(nav_data);
  }
}

bool Main::imuChanged(const Sensors& old_data, const Sensors& new_data)
{
  for (unsigned int i = 0; i < new_data.imu.size(); ++i)
    if (new_data.imu[i].gyr.timestamp != old_data.imu[i].gyr.timestamp ||
        new_data.imu[i].acc.timestamp != old_data.imu[i].acc.timestamp)
      return true;
  return false;
}

bool Main::proxiChanged(const Sensors& old_data, const Sensors& new_data)
{
  for (unsigned int i = 0; i < new_data.proxy.size(); ++i)
    // TODO(Brano): Timestamp proxi data in data::Sensors
    if (new_data.proxy[i].val != old_data.proxy[i].val)
      return true;
  return false;
}

inline bool Main::stripeCntChanged(const Sensors& old_data, const Sensors& new_data)
{
  return new_data.stripe_cnt.timestamp != old_data.stripe_cnt.timestamp;
}

}}  // namespace hyped::navigation

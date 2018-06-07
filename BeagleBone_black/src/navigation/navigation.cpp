/*
 * Author: Adithya Sireesh, Uday Patel
 * Organisation: HYPED
 * Date: 10/02/2018
 * Description: Navigation class functionality definition.
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

#include "navigation.hpp"
#include <math.h>

namespace hyped {
namespace navigation {

Navigation::Navigation() : prev_angular_velocity_(0 , NavigationVector())
{
  for (int i = 0; i < Sensors::kNumImus; i++) {
      acceleration_filter_[i].configure(NavigationVector(),
                                        NavigationVector(),
                                        NavigationVector());
      gyro_filter_[i].configure(NavigationVector(),
                                NavigationVector(),
                                NavigationVector());
  }

  for (auto filter: proximity_filter_)
    filter.configure(0, 0, 0);
}

NavigationType Navigation::get_accleration()
{
  return accleration_[0];
}

NavigationType Navigation::get_velocity()
{
  return velocity_[0];
}

NavigationType Navigation::get_displacement()
{
  return displacement_[0];
}


void Navigation::update(ImuArray imus)
{
  // TODO(Brano,Adi): Gyro update. (Data format should change first.)
  for (int i = 0; i < data::Sensors::kNumImus; i++) {
    imus[i].acc.value = acceleration_filter_[i].filter(imus[i].acc.value);
    imus[i].gyr.value = gyro_filter_[i].filter(imus[i].gyr.value);
  }

  NavigationVector avg(0);
  for (const auto& imu : imus) avg += imu.acc.value;

  avg /= imus.size();
  // TODO(Brano,Adi): Change the timestamping strategy
  this->acclerometer_update(DataPoint<NavigationVector>(imus[0].acc.timestamp, avg));
}

void Navigation::update(ImuArray imus, ProximityArray proxis)
{
  update(imus);
  // TODO(Brano,Adi): Proximity updates. (Data format needs to be changed first.)
}

void Navigation::update(ImuArray imus, DataPoint<uint32_t> stripe_count)
{
  update(imus);
  // TODO(Brano,Adi): Do something with stripe cnt timestamp as well?
  stripe_counter_update(stripe_count.value);
}

void Navigation::update(ImuArray imus, ProximityArray proxis, DataPoint<uint32_t> stripe_count)
{
  update(imus, proxis);
  stripe_counter_update(stripe_count.value);
}

void Navigation::gyro_update(DataPoint<NavigationVector> angular_velocity)
{
  double theta             = prev_angular_velocity_.value.norm();
  double angle_of_rotation = (angular_velocity.timestamp - prev_angular_velocity_.timestamp)
                             * theta/2;
  double scalar_part  = cos(angle_of_rotation);
  auto   vector_part  = (prev_angular_velocity_.value/theta)*sin(angle_of_rotation);

  Quaternion<int16_t> rotation_quaternion(scalar_part, vector_part);
  orientation_          *= rotation_quaternion;
  prev_angular_velocity_ = angular_velocity;
}

void Navigation::acclerometer_update(DataPoint<NavigationVector> acceleration)
{
  accleration_  = acceleration.value;
  auto velocity = acceleration_integrator_.update(acceleration);
  velocity_     = velocity.value;
  displacement_ = velocity_integrator_.update(velocity).value;
}

void Navigation::proximity_orientation_update()
{
  // TODO(Adi): Calculate SLERP (Point 2 of the FDP).
}

void Navigation::proximity_displacement_update()
{
  // TODO(Adi): Calculate displacement from proximity. (Point 7)
}

void Navigation::stripe_counter_update(uint16_t count)
{}

}}  // namespace hyped::navigation

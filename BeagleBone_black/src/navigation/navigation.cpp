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

NavigationType Navigation::getAcceleration()
{
  return acceleration_[0];
}

NavigationType Navigation::getVelocity()
{
  return velocity_[0];
}

NavigationType Navigation::getDisplacement()
{
  return displacement_[0];
}

NavigationType Navigation::getEmergencyBrakingDistance()
{
  // TODO(Brano): Account for actuation delay and/or communication latency?
  return velocity_[0]*velocity_[0] / kEmergencyDeceleration;
}


void Navigation::update(ImuArray imus)
{
  int num_operational = 0;
  uint32_t acc_time = 0, gyr_time = 0;  // TODO(Brano): Deal with overflows (individual and sum)
  NavigationVector acc(0), gyr(0);
  for (int i = 0; i < imus.size(); ++i) {
    if (imus[i].operational) {
      ++num_operational;
      acc_time += imus[i].acc.timestamp;
      acc      += acceleration_filter_[i].filter(imus[i].acc.value);
      gyr_time += imus[i].gyr.timestamp;
      gyr      += gyro_filter_[i].filter(imus[i].gyr.value);
    }
  }

  // TODO(Brano): Check num_operational for crit. failure

  accelerometerUpdate(DataPoint<NavigationVector>(acc_time/num_operational, acc/num_operational));
           gyroUpdate(DataPoint<NavigationVector>(gyr_time/num_operational, gyr/num_operational));
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
  stripeCounterUpdate(stripe_count.value);
}

void Navigation::update(ImuArray imus, ProximityArray proxis, DataPoint<uint32_t> stripe_count)
{
  update(imus, proxis);
  stripeCounterUpdate(stripe_count.value);
}

void Navigation::gyroUpdate(DataPoint<NavigationVector> angular_velocity)
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

void Navigation::accelerometerUpdate(DataPoint<NavigationVector> acceleration)
{
  acceleration_  = acceleration.value;
  auto velocity = acceleration_integrator_.update(acceleration);
  velocity_     = velocity.value;
  displacement_ = velocity_integrator_.update(velocity).value;
}

void Navigation::proximityOrientationUpdate()
{
  // TODO(Adi): Calculate SLERP (Point 2 of the FDP).
}

void Navigation::proximityDisplacementUpdate()
{
  // TODO(Adi): Calculate displacement from proximity. (Point 7)
}

void Navigation::stripeCounterUpdate(uint16_t count)
{}

}}  // namespace hyped::navigation

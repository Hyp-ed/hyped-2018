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

Navigation::Navigation()
    : prev_angular_velocity_(0 , NavigationVector()),
      state_(NavigationState::kCalibrating),
      num_gravity_samples_(0),
      g_(0),
      num_gyro_samples_(0)
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

NavigationState Navigation::getState()
{
  return state_;
}

bool Navigation::finishCalibration(Barrier navigation_motors_sync)
{
  if (state_ != NavigationState::kReady)
    return false;

  // Finalize calibration, update state, hit the barrier
  g_ /= num_gravity_samples_;
  for (NavigationVector& v : gyro_offsets_)
    v /= num_gyro_samples_;
  
  // Update state
  state_ = NavigationState::kOperational;

  // Hit the barrier to sync with motors
  navigation_motors_sync.wait();

  return true;
}


void Navigation::update(ImuArray imus)
{
  if (state_ == NavigationState::kCalibrating || state_ == NavigationState::kReady) {
    calibrationUpdate(imus);
  } else {
    // TODO(Brano,Adi): Gyro update. (Data format should change first.)
    for (int i = 0; i < data::Sensors::kNumImus; i++) {
      imus[i].acc.value = acceleration_filter_[i].filter(imus[i].acc.value);
      imus[i].gyr.value = gyro_filter_[i].filter(imus[i].gyr.value);
    }

    NavigationVector acc_avg(0), gyr_avg(0);
    for (const auto& imu : imus) {
      acc_avg += imu.acc.value;
      gyr_avg += imu.gyr.value;  // TODO(Brano,Adi): Check if gyro can be averaged like this
    }
    acc_avg /= imus.size();
    gyr_avg /= imus.size();

    // TODO(Brano,Adi): Change the timestamping strategy
    this->accelerometerUpdate(DataPoint<NavigationVector>(imus[0].acc.timestamp, acc_avg));
    this->gyroUpdate(DataPoint<NavigationVector>(imus[0].gyr.timestamp, gyr_avg));
  }
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

void Navigation::calibrationUpdate(ImuArray imus)
{
  for (unsigned int i = 0; i < data::Sensors::kNumImus; ++i) {
    g_ += imus[i].acc.value;
    gyro_offsets_[i] += imus[i].gyr.value;
  }
  ++num_gravity_samples_;
  ++num_gyro_samples_;
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

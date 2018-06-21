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

#include <algorithm>  // std::min
#include <cmath>

namespace hyped {
namespace navigation {

float proxiMean(const Proximity* const a, const Proximity* const b)
{
  if (a->operational && b->operational)
    return (float)(a->val + b->val)/2.0;
  if (a->operational)
    return a->val;
  if (b->operational)
    return b->val;
  return -1;
}

Navigation::Navigation(Barrier& post_calibration_barrier)
    : post_calibration_barrier_(post_calibration_barrier),
      status_(ModuleStatus::kStart),
      is_calibrating_(false),
      num_gravity_samples_(0),
      g_(0),
      num_gyro_samples_(0),
      acceleration_(0),  // TODO(Brano): Should this be g or 0?
      velocity_(0),
      displacement_(0),
      prev_angular_velocity_(0 , NavigationVector()),
      orientation_(1, 0, 0, 0)
{
  for (int i = 0; i < Sensors::kNumImus; i++) {
    // TODO(Brano,Uday): Properly initialise filters (with std dev of sensors and stuff)
    acceleration_filter_[i].configure(NavigationVector(),
                                      NavigationVector(),
                                      NavigationVector());
    gyro_filter_[i].configure(NavigationVector(),
                              NavigationVector(),
                              NavigationVector());
  }

  for (auto filter: proximity_filter_)
    filter.configure(0, 0, 0);

  status_ = ModuleStatus::kInit;
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

ModuleStatus Navigation::getStatus()
{
  return status_;
}

bool Navigation::startCalibration()
{
  if (is_calibrating_)
    return true;
  if (status_ != ModuleStatus::kInit)
    return false;

  is_calibrating_ = true;
  return true;
}

bool Navigation::finishCalibration()
{
  if (!is_calibrating_ || status_ != ModuleStatus::kReady)
    return false;

  // Finalize calibration
  g_ /= num_gravity_samples_;
  for (NavigationVector& v : gyro_offsets_)
    v /= num_gyro_samples_;

  // Update state
  is_calibrating_ = false;

  // Hit the barrier to sync with motors
  post_calibration_barrier_.wait();

  return true;
}


std::array<NavigationType, 3> Navigation::getNearestStripeDists()
{
  std::array<NavigationType, 3> arr;
  for (unsigned int i = 0; i < arr.size(); ++i)
    arr[i] = kStripeLocations[std::min(stripe_count_ + i, (unsigned int)kStripeLocations.size())]
             - getDisplacement();
  return arr;
}

void Navigation::update(DataPoint<ImuArray> imus)
{
  int num_operational = 0;
  NavigationVector acc(0), gyr(0);
  for (int i = 0; i < imus.value.size(); ++i) {
    if (imus.value[i].operational) {
      ++num_operational;
      acc += acceleration_filter_[i].filter(imus.value[i].acc);
      gyr += gyro_filter_[i].filter(imus.value[i].gyr);
    }
  }

  // TODO(Brano): Check num_operational for crit. failure

  accelerometerUpdate(DataPoint<NavigationVector>(imus.timestamp, acc/num_operational));
           gyroUpdate(DataPoint<NavigationVector>(imus.timestamp, gyr/num_operational));
}

void Navigation::update(DataPoint<ImuArray> imus, ProximityArray proxis)
{
  update(imus);

  Proximities ground, rail;
  // TODO(Brano,Martin): Make sure proxis are in correct order (define index constants)
  ground.fr = proxiMean(proxis[0],  proxis[1]);
  ground.rr = proxiMean(proxis[2],  proxis[3]);
  ground.rl = proxiMean(proxis[4],  proxis[5]);
  ground.fl = proxiMean(proxis[6],  proxis[7]);
  rail.fr   = proxiMean(proxis[8],  proxis[9]);
  rail.rr   = proxiMean(proxis[10], proxis[11]);
  rail.rl   = proxiMean(proxis[12], proxis[13]);
  rail.fl   = proxiMean(proxis[14], proxis[15]);
  // TODO(Brano): Check for crit. failure
  proximityDisplacementUpdate(ground, rail);
  proximityOrientationUpdate(ground, rail);
}

void Navigation::update(DataPoint<ImuArray> imus, DataPoint<uint32_t> stripe_count)
{
  update(imus);
  // TODO(Brano,Adi): Do something with stripe cnt timestamp as well?
  stripeCounterUpdate(stripe_count.value);
}

void Navigation::update(DataPoint<ImuArray> imus,
                        ProximityArray proxis,
                        DataPoint<uint32_t> stripe_count)
{
  update(imus, proxis);
  stripeCounterUpdate(stripe_count.value);
}

void Navigation::calibrationUpdate(ImuArray imus)
{
  // Online mean algorithm
  ++num_gyro_samples_;
  for (unsigned int i = 0; i < data::Sensors::kNumImus; ++i) {
    ++num_gravity_samples_;
    g_ = g_ + (imus[i].acc - g_)/num_gravity_samples_;
    gyro_offsets_[i] = gyro_offsets_[i] + (imus[i].gyr - gyro_offsets_[i])/num_gyro_samples_;
  }

  if (num_gravity_samples_ > kMinNumCalibrationSamples
      && num_gyro_samples_ > kMinNumCalibrationSamples)
    status_ = ModuleStatus::kReady;
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

void Navigation::proximityOrientationUpdate(Proximities ground, Proximities rail)
{
  // TODO(Adi): Calculate SLERP (Point 2 of the FDP).
}

void Navigation::proximityDisplacementUpdate(Proximities ground, Proximities rail)
{
  // TODO(Adi): Calculate displacement from proximity. (Point 7)
}

void Navigation::stripeCounterUpdate(uint16_t count)
{
  // TODO(Brano): Check for errors (e.g. missed stripes)
  // TODO(Brano): Update displacement and velocity
  stripe_count_ = count;
}

}}  // namespace hyped::navigation

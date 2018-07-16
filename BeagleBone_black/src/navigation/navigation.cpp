/*
 * Author: Adithya Sireesh, Uday Patel, Brano Pilnan and Ragnor Comerford
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

constexpr float kWheelDiameter = 0.0608;
const Navigation::Settings Navigation::kDefaultSettings;
float proxiMean(const Proximity* const a, const Proximity* const b)
{
  if (a->operational && b->operational)
    return static_cast<float>(a->val + b->val)/2.0;
  if (a->operational)
    return a->val;
  if (b->operational)
    return b->val;
  return -1;
}

Navigation::Navigation(Barrier& post_calibration_barrier,
                       Logger& log,
                       const Settings& settings)
    : post_calibration_barrier_(post_calibration_barrier),
      log_(log),
      settings_(settings),
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
{}

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

void Navigation::init(SensorCalibration sc, Sensors readings)
{
  for (int i = 0; i < Sensors::kNumImus; i++) {
    // TODO(Brano,Uday): Properly initialise filters (with std dev of sensors and stuff)
    acceleration_filter_[i].configure(readings.imu.value[i].acc,
                                      sc.imu_variance[i][0].sqrt(),
                                      NavigationVector(0.1));
    gyro_filter_[i].configure(readings.imu.value[i].gyr,
                              sc.imu_variance[i][0].sqrt(),
                              NavigationVector(0.1));
  }

  for (int i = 0; i < Sensors::kNumProximities; ++i) {
    proximity_filter_[i].configure(readings.proxi_front.value[i].val,
                                   sqrt(sc.proxi_front_variance[i]),
                                   0.1);
    proximity_filter_[i + Sensors::kNumProximities].configure(readings.proxi_back.value[i].val,
                                                              sqrt(sc.proxi_back_variance[i]),
                                                              0.1);
  }

  log_.DBG("NAV",
      "After init: a=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, %.3f), d=(%.3f, %.3f, %.3f)",
      acceleration_[0], acceleration_[1], acceleration_[2],
      velocity_[0], velocity_[1], velocity_[2],
      displacement_[0], displacement_[1], displacement_[2]);

  status_ = ModuleStatus::kInit;
}


std::array<NavigationType, 3> Navigation::getNearestStripeDists()
{
  std::array<NavigationType, 3> arr;
  for (unsigned int i = 0; i < arr.size(); ++i)
    arr[i] = kStripeLocations[std::min(stripe_count_ + i, (unsigned int)kStripeLocations.size())]
             - getDisplacement();
  return arr;
}

void Navigation::update(NavigationInput input)
{
  if (input.imus != nullptr) {
    updateImus(*input.imus);
  }
  if (input.proxis != nullptr && !is_calibrating_) {
    updateProxis(*input.proxis);
  }
  if (input.sc != nullptr && !is_calibrating_) {
    stripeCounterUpdate(*input.sc);
  }
  if (input.oe != nullptr && !is_calibrating_) {
    opticalEncoderUpdate(*input.oe);
  }
}

void Navigation::updateImus(DataPoint<ImuArray> imus)
{
  for (unsigned int i = 0; i < imus.value.size(); ++i) {
    log_.DBG3("NAV", "Before filtering: a[%d]=(%.3f, %.3f, %.3f), omega[%d]=(%.3f, %.3f, %.3f)",
        i, imus.value[i].acc[0], imus.value[i].acc[1], imus.value[i].acc[2],
        i, imus.value[i].gyr[0], imus.value[i].gyr[1], imus.value[i].gyr[2]);
    imus.value[i].acc = acceleration_filter_[i].filter(imus.value[i].acc);
    imus.value[i].gyr = gyro_filter_[i].filter(imus.value[i].gyr);
    log_.DBG3("NAV", " After filtering: a[%d]=(%.3f, %.3f, %.3f), omega[%d]=(%.3f, %.3f, %.3f)",
        i, imus.value[i].acc[0], imus.value[i].acc[1], imus.value[i].acc[2],
        i, imus.value[i].gyr[0], imus.value[i].gyr[1], imus.value[i].gyr[2]);
  }

  int num_operational = 0;
  NavigationVector acc(0), gyr(0);
  for (unsigned int i = 0; i < imus.value.size(); ++i) {
    if (imus.value[i].operational) {
      ++num_operational;
      acc += imus.value[i].acc;
      gyr += imus.value[i].gyr;
    }
  }

  if (num_operational < 2) {
    status_ = ModuleStatus::kCriticalFailure;
    log_.ERR("NAV", "Critical failure: num operational IMUs = %d < 2", num_operational);
  }

  if (is_calibrating_) {
    calibrationUpdate(imus.value);
  } else {
    accelerometerUpdate(DataPoint<NavigationVector>(imus.timestamp, acc/num_operational));
             gyroUpdate(DataPoint<NavigationVector>(imus.timestamp, gyr/num_operational));
  }
}

void Navigation::updateProxis(ProximityArray proxis)
{
  Proximities ground, rail;
  // TODO(Brano,Martin): Make sure proxis are in correct order (define index constants)
  int num_ground_fail = 0;
  if ( (ground.fr = proxiMean(proxis[6],  proxis[7]) ) < 0 ) ++num_ground_fail;
  if ( (ground.rr = proxiMean(proxis[8],  proxis[9]) ) < 0 ) ++num_ground_fail;
  if ( (ground.rl = proxiMean(proxis[14], proxis[15])) < 0 ) ++num_ground_fail;
  if ( (ground.fl = proxiMean(proxis[0],  proxis[1]) ) < 0 ) ++num_ground_fail;
  rail.fr   = proxiMean(proxis[4],  proxis[5]);
  rail.rr   = proxiMean(proxis[10], proxis[11]);
  rail.rl   = proxiMean(proxis[12], proxis[13]);
  rail.fl   = proxiMean(proxis[2],  proxis[3]);

  // Check for crit. failure
  if (num_ground_fail > 1) {
    status_ = ModuleStatus::kCriticalFailure;
    log_.ERR("NAV", "Critical failure: num failed ground proxi points = %d", num_ground_fail);
    return;
  }
  if ((rail.fr < 0 && rail.fl < 0) || (rail.rr < 0 && rail.rl < 0)) {
    status_ = ModuleStatus::kCriticalFailure;
    log_.ERR("NAV", "Critical failure: insufficient rail proxis");
    return;
  }

  proximityDisplacementUpdate(ground, rail);
  proximityOrientationUpdate(ground, rail);
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
  log_.DBG2("NAV",
      "Before accl update: a=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, %.3f), d=(%.3f, %.3f, %.3f)",
      acceleration_[0], acceleration_[1], acceleration_[2],
      velocity_[0], velocity_[1], velocity_[2],
      displacement_[0], displacement_[1], displacement_[2]);
  acceleration_  = acceleration.value;
  auto velocity = acceleration_integrator_.update(acceleration);
  velocity_     = velocity.value;
  displacement_ = velocity_integrator_.update(velocity).value;
  log_.DBG2("NAV",
      " After accl update: a=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, %.3f), d=(%.3f, %.3f, %.3f)",
      acceleration_[0], acceleration_[1], acceleration_[2],
      velocity_[0], velocity_[1], velocity_[2],
      displacement_[0], displacement_[1], displacement_[2]);
}

void Navigation::proximityOrientationUpdate(Proximities ground, Proximities rail)
{
  // TODO(Adi): Calculate SLERP (Point 2 of the FDP).
}

void Navigation::proximityDisplacementUpdate(Proximities ground, Proximities rail)
{
  log_.DBG2("NAV",
      "Before proxi displ update: a=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, %.3f), d=(%.3f, %.3f, %.3f)",
      acceleration_[0], acceleration_[1], acceleration_[2],
      velocity_[0], velocity_[1], velocity_[2],
      displacement_[0], displacement_[1], displacement_[2]);
  NavigationVector proxi_displ = displacement_;
  // TODO(Brano): Make this a weighted average based on the actual positions of the 4 sensors with
  //              respect to IMUs
  proxi_displ[2] = (ground.fr + ground.rr + ground.rl + ground.fl) / 4.0;
  // Change from mm to m and subtract the stationary z-axis displacement
  // TODO(Brano): Update the z-axis displacement
  proxi_displ[2] = proxi_displ[2]/1000.0 - 0.1;
  // The y-axis points to the left and displacement 0 is when right and left proxis are equal
  // TODO(Brano): Make this a weighted average based on the actual proxi positions w.r.t. IMUs
  proxi_displ[1] = ((rail.fl - rail.fr) + (rail.rl - rail.rr)) / 2.0;
  proxi_displ[1] = proxi_displ[1]/1000.0;

  // Update displacement
  displacement_ = (1 - settings_.prox_displ_w)*displacement_ + settings_.prox_displ_w*proxi_displ;

  // Update velocity
  DataPoint<Vector<NavigationType, 2>> dp(
      prev_angular_velocity_.timestamp,  /* IMUs always updated */
      Vector<NavigationType, 2>({proxi_displ[1], proxi_displ[2]}));
  auto proxi_vel = proxi_differentiator_.update(dp).value;

  velocity_[1] = (1 - settings_.prox_vel_w)*velocity_[1] + settings_.prox_vel_w*proxi_vel[0];
  velocity_[2] = (1 - settings_.prox_vel_w)*velocity_[2] + settings_.prox_vel_w*proxi_vel[1];

  log_.DBG2("NAV",
      " After proxi displ update: a=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, %.3f), d=(%.3f, %.3f, %.3f)",
      acceleration_[0], acceleration_[1], acceleration_[2],
      velocity_[0], velocity_[1], velocity_[2],
      displacement_[0], displacement_[1], displacement_[2]);
}

void Navigation::stripeCounterUpdate(StripeCounter sc)
{
  log_.DBG2("NAV",
      "Before stripe update: a=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, %.3f), d=(%.3f, %.3f, %.3f)",
      acceleration_[0], acceleration_[1], acceleration_[2],
      velocity_[0], velocity_[1], velocity_[2],
      displacement_[0], displacement_[1], displacement_[2]);
  // TODO(Brano): Update displacement and velocity

  // TODO(Brano): Change this once 2nd Keyence is added
  if (!sc.operational) {
    status_ = ModuleStatus::kCriticalFailure;
    log_.ERR("NAV", "Critical failure: stripe counter down");
    return;
  }
  auto dists = getNearestStripeDists();
  if (std::abs(dists[0]) < std::abs(dists[1]) || std::abs(dists[2]) < std::abs(dists[1])) {
    // Ideally, we'd have dists[1]==0 but if dists[1] is not the closest stripe, something has
    // definitely gone wrong.
    status_ = ModuleStatus::kCriticalFailure;
    log_.ERR("NAV",
        "Critical failure: missed stripe (oldCnt=%d, newCnt=%d, nearestStripes=[%f, %f, %f])",
        stripe_count_, sc.count.value, dists[0], dists[1], dists[2]);
    return;
  }

  stripe_count_ = sc.count.value;
  DataPoint<NavigationType> dp(sc.count.timestamp, kStripeLocations[stripe_count_]);

  // Update x-axis (forwards) displacement
  displacement_[0] = (1 - settings_.strp_displ_w) * displacement_[0] +
                          settings_.strp_displ_w  * dp.value;

  // Update x-axis velocity
  velocity_[0] = (1 - settings_.strp_vel_w) * velocity_[0] +
                      settings_.strp_vel_w  * stripe_differentiator_.update(dp).value;

  log_.DBG2("NAV",
      " After stripe update: a=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, %.3f), d=(%.3f, %.3f, %.3f)",
      acceleration_[0], acceleration_[1], acceleration_[2],
      velocity_[0], velocity_[1], velocity_[2],
      displacement_[0], displacement_[1], displacement_[2]);
}

void Navigation::opticalEncoderUpdate(StripeCounter oe)
{
  log_.DBG2("NAV",
      "Before stripe update: a=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, %.3f), d=(%.3f, %.3f, %.3f)",
      acceleration_[0], acceleration_[1], acceleration_[2],
      velocity_[0], velocity_[1], velocity_[2],
      displacement_[0], displacement_[1], displacement_[2]);
  // TODO(Brano): Update displacement and velocity

  DataPoint<NavigationType> dp(oe.count.timestamp, oe.count.value*M_PI*kWheelDiameter);

  // Update x-axis (forwards) displacement
  displacement_[0] = (1 - settings_.strp_displ_w) * displacement_[0] +
                          settings_.strp_displ_w  * dp.value;

  // Update x-axis velocity
  velocity_[0] = (1 - settings_.strp_vel_w) * velocity_[0] +
                      settings_.strp_vel_w  * stripe_differentiator_.update(dp).value;

  log_.DBG2("NAV",
      " After stripe update: a=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, %.3f), d=(%.3f, %.3f, %.3f)",
      acceleration_[0], acceleration_[1], acceleration_[2],
      velocity_[0], velocity_[1], velocity_[2],
      displacement_[0], displacement_[1], displacement_[2]);
}
}}  // namespace hyped::navigation

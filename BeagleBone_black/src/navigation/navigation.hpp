/*
 * Author: Adithya Sireesh
 * Organisation: HYPED
 * Date: 10/02/2018
 * Description: Navigation class skeleton.
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

#ifndef BEAGLEBONE_BLACK_NAVIGATION_NAVIGATION_HPP_
#define BEAGLEBONE_BLACK_NAVIGATION_NAVIGATION_HPP_

#include <array>
#include <cstdint>

#include "data/data.hpp"
#include "utils/math/integrator.hpp"
#include "utils/math/kalman.hpp"
#include "utils/math/quaternion.hpp"
#include "utils/math/vector.hpp"

namespace hyped {

using data::DataPoint;
using data::Imu;
using data::Proximity;
using data::Sensors;
using data::NavigationType;
using data::NavigationVector;
using utils::math::Integrator;
using utils::math::Kalman;
using utils::math::Quaternion;
using utils::math::Vector;

namespace navigation {

class Navigation {
 public:
  typedef std::array<Imu,       Sensors::kNumImus>        ImuArray;
  typedef std::array<Proximity, Sensors::kNumProximities> ProximityArray;
  friend class Main;

  Navigation();

  /**
   * @brief Get the accleration value
   *
   * @return uint16_t Returns the forward component of acceleration vector (negative when
   *                  decelerating)
   */
  NavigationType get_accleration();
  /**
   * @brief Get the velocity value
   *
   * @return uint16_t Returns the forward component of velocity vector
   */
  NavigationType get_velocity();
  /**
   * @brief Get the displacement value
   *
   * @return uint16_t Returns the forward component of displacement vector
   */
  NavigationType get_displacement();

 private:
  /**
   * @brief Updates navigation values based on new IMU reading. This should be called when new IMU
   *        reading is available but no other data has been updated.
   *
   * @param[in] imus Array of IMU readings
   */
  void update(ImuArray imus);
  /**
   * @brief Updates navigation based on new IMU and proxi readings. Should be called when IMU and
   *        proxi have been updated but there is no update from stripe counter.
   *
   * @param[in] imus   Array of IMU readings
   * @param[in] proxis Array of proximity readings
   */
  void update(ImuArray imus, ProximityArray proxis);
  /**
   * @brief Updates navigation based on new IMU and stripe counter readings. Should be called when
   *        IMU and stripe counter have been updated but there is no update from proximity sensors.
   *
   * @param imus         Array of IMU readings
   * @param stripe_count Stripe counter reading
   */
  void update(ImuArray imus, data::StripeCount stripe_count);
  /**
   * @brief Updates navigation based on new IMU and stripe counter readings. Should be called when
   *        IMU, proximity sensors, and stripe counter have all been updated.
   *
   * @param imus         Array of IMU readings
   * @param[in] proxis   Array of proximity readings
   * @param stripe_count Stripe counter reading
   */
  void update(ImuArray imus, ProximityArray proxis, data::StripeCount stripe_count);

  void gyro_update(DataPoint<NavigationVector> angular_velocity);  // Point number 1
  void acclerometer_update(DataPoint<NavigationVector> acceleration);  // Points 3, 4, 5, 6
  void proximity_orientation_update();  // Point number 7
  void proximity_displacement_update();  // Point number 7
  void stripe_counter_update(uint16_t count);  // Point number 7

  // Most up-to-date values of pod's acceleration, velocity and displacement in 3D; used for output
  NavigationVector accleration_;
  NavigationVector velocity_;
  NavigationVector displacement_;

  // Internal data that is not published
  DataPoint<NavigationVector> prev_angular_velocity_;  // To calculate how much has the pod rotated
  Quaternion<int16_t> orientation_;  // Pod's orientation is updated with every gyro reading

  // Filters for reducing noise in sensor data before processing the data in any other way
  std::array<Kalman<NavigationVector>, Sensors::kNumImus> acceleration_filter_;  // One for each IMU
  std::array<Kalman<NavigationVector>, Sensors::kNumImus> gyro_filter_;          // One for each IMU
  std::array<Kalman<uint8_t>, Sensors::kNumProximities>   proximity_filter_;

  Integrator<NavigationVector> acceleration_integrator_;  // Acceleration to velocity
  Integrator<NavigationVector> velocity_integrator_;      // Velocity to displacement
};

}}  // namespace hyped::navigation

#endif  // BEAGLEBONE_BLACK_NAVIGATION_NAVIGATION_HPP_

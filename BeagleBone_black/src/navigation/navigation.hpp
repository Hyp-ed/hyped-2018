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
using utils::math::Integrator;
using utils::math::Kalman;
using utils::math::Quaternion;
using utils::math::Vector;

namespace navigation {

class Navigation {
 public:
  Navigation();

  /**
   * @brief Updates navigation values based on new IMU reading. This should be called when new IMU
   *        reading is available but no other data has been updated.
   *
   * @param[in] imus Array of IMU readings
   */
  void update(std::array<data::Imu, data::Sensors::kNumImus> imus);
  /**
   * @brief Updates navigation based on new IMU and proxi readings. Should be called when IMU and
   *        proxi have been updated but there is no update from stripe counter.
   *
   * @param[in] imus   Array of IMU readings
   * @param[in] proxis Array of proximity readings
   */
  void update(std::array<data::Imu, data::Sensors::kNumImus> imus,
              std::array<data::Proximity, data::Sensors::kNumProximities> proxis);
  /**
   * @brief Updates navigation based on new IMU and stripe counter readings. Should be called when
   *        IMU and stripe counter have been updated but there is no update from proximity sensors.
   *
   * @param imus         Array of IMU readings
   * @param stripe_count Stripe counter reading
   */
  void update(std::array<data::Imu, data::Sensors::kNumImus> imus, data::StripeCount stripe_count);
  /**
   * @brief Updates navigation based on new IMU and stripe counter readings. Should be called when
   *        IMU, proximity sensors, and stripe counter have all been updated.
   *
   * @param imus         Array of IMU readings
   * @param[in] proxis   Array of proximity readings
   * @param stripe_count Stripe counter reading
   */
  void update(std::array<data::Imu, data::Sensors::kNumImus> imus,
              std::array<data::Proximity, data::Sensors::kNumProximities> proxis,
              data::StripeCount stripe_count);
  /**
   * @brief Get the accleration value
   *
   * @return uint16_t Returns the forward component of acceleration vector (negative when
   *                  decelerating)
   */
  int16_t get_accleration();
  /**
   * @brief Get the velocity value
   *
   * @return uint16_t Returns the forward component of velocity vector
   */
  int16_t get_velocity();
  /**
   * @brief Get the displacement value
   *
   * @return uint16_t Returns the forward component of displacement vector
   */
  int16_t get_displacement();

 private:
  // TODO(Uday,Brano,Adi): Add the arguments for the private methods
  // TODO(Uday,Brano,Adi): Change the return values for the private methods to vector/quaternion

  void gyro_update(DataPoint<Vector<int16_t, 3>> angular_velocity);  // Point number 1
  void acclerometer_update(DataPoint<Vector<int16_t, 3>> acceleration);  // Points 3, 4, 5, 6
  void proximity_orientation_update();  // Point number 7
  void proximity_displacement_update();  // Point number 7
  void stripe_counter_update(uint16_t count);  // Point number 7

  // TODO(Uday,Brano,Adi): Change the data types of the data members

  Vector<int16_t, 3> accleration_;
  Vector<int16_t, 3> velocity_;
  Vector<int16_t, 3> displacement_;
  Quaternion<int16_t> orientation_;
  int stripe_count_;

  // TODO(ALL): Decide the type
  std::array<Kalman<Vector<int16_t, 3>>, data::Sensors::kNumImus> acceleration_filter_;
  std::array<Kalman<Vector<int16_t, 3>>, data::Sensors::kNumImus> gyro_filter_;
  std::array<Kalman<uint8_t>, data::Sensors::kNumProximities> proximity_filter_;

  Integrator<Vector<int16_t, 3>> acceleration_integrator_;
  Integrator<Vector<int16_t, 3>> velocity_integrator_;
};

}}  // namespace hyped::navigation

#endif  // BEAGLEBONE_BLACK_NAVIGATION_NAVIGATION_HPP_

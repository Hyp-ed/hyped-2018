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
#include <cmath>
#include <string>
#include <fstream>

#include "data/data.hpp"
#include "utils/concurrent/barrier.hpp"
#include "utils/logger.hpp"
#include "utils/math/differentiator.hpp"
#include "utils/math/integrator.hpp"
#include "utils/math/kalman.hpp"
#include "utils/math/quaternion.hpp"
#include "utils/math/vector.hpp"
#include "utils/system.hpp"

namespace hyped {

using data::DataPoint;
using data::Imu;
using data::ModuleStatus;
using data::NavigationType;
using data::NavigationVector;
#ifdef PROXI
using data::Proximity;
#endif
using data::SensorCalibration;
using data::Sensors;
using data::StripeCounter;
using utils::concurrent::Barrier;
using utils::Logger;
using utils::math::Differentiator;
using utils::math::Integrator;
using utils::math::Kalman;
using utils::math::Quaternion;
using utils::math::Vector;
using utils::System;

namespace navigation {

// Positions (in m) of proximity sensors w.r.t. IMUs
// TODO(Brano): Update the positions once stuff is mounted
const NavigationVector kGroundProxiFR({ 1, -0.5, -0.1});
const NavigationVector kGroundProxiFL({ 1,  0.5, -0.1});
const NavigationVector kGroundProxiRL({-2,  0.5, -0.1});
const NavigationVector kGroundProxiRR({-2, -0.5, -0.1});
const NavigationVector kRailProxiFR({ 1, -0.1, -0.1});
const NavigationVector kRailProxiFL({ 1,  0.1, -0.1});
const NavigationVector kRailProxiRL({-2,  0.1, -0.1});
const NavigationVector kRailProxiRR({-2, -0.1, -0.1});

constexpr NavigationType kEmergencyDeceleration = 24;  // m/s^2
constexpr std::array<NavigationType, 42> kStripeLocations = {0.0,
      30.48,   60.96,   91.44,  121.92,  152.4,  182.88,  213.36,  243.84,  274.32,  304.8,
     335.28,  365.76,  396.24,  426.72,  457.2,  487.68,  518.16,  548.64,  579.12,  609.6,
     640.08,  670.56,  701.04,  731.52,  762.0,  792.48,  822.96,  853.44,  883.92,  914.4,
     944.88,  975.36, 1005.84, 1036.32, 1066.8, 1097.28, 1127.76, 1158.24, 1188.72, 1219.2,
    1249.68};

class Navigation {
  friend class Main;

 public:
  typedef std::array<Imu,           Sensors::kNumImus>          ImuArray;
#ifdef PROXI
  typedef std::array<Proximity*,    2*Sensors::kNumProximities> ProximityArray;
#endif
  typedef std::array<StripeCounter, Sensors::kNumKeyence>       StripeCounterArray;
  struct Settings {
#ifdef PROXI
    bool proxi_displ_enable = false;  // Needs updated proxi positions
    bool proxi_orient_enable = false;  // Needs updated proxi positions
#endif
    bool gyro_enable = false;  // Not fully implemented (rotate a)
    bool opt_enc_enable = false;  // Not implemented
    bool keyence_enable = true;
    // TODO(Brano): Change the default values
    float prox_orient_w = 0.1;  ///< Weight (from [0,1]) of proxi vs imu in orientation calculation
    float prox_displ_w = 0.1;  ///< Weight (from [0,1]) of proxi vs imu in displacement calculation
    float strp_displ_w = 1.0;  ///< Weight [0,1] of stripe count vs imu in displacement calculation
    float prox_vel_w = 0.01;  ///< Weight (from [0,1]) of proxi vs imu in velocity calculation
    float strp_vel_w = 0.0;  ///< Weight [0,1]  of stripe count vs imu in velocity calculation
  };
  struct Input {
    DataPoint<ImuArray> *imus = nullptr;
#ifdef PROXI
    ProximityArray *proxis = nullptr;
#endif
    StripeCounterArray *sc = nullptr;
    array<float, Sensors::kNumOptEnc> *optical_enc_distance = nullptr;
  };
  struct FullOutput {
    const ModuleStatus*     status;
    const bool*             is_calibrating;
    const int*              num_gravity_samples;
    const std::array<NavigationVector, Sensors::kNumImus>* g;  // Acc offsets (gravitational acc)
    const int*              num_gyro_samples;
    const std::array<NavigationVector, Sensors::kNumImus>* gyro_offsets;

    // Most up-to-date values of pod's acceleration, velocity and displacement in 3D
    const NavigationVector* acceleration;
    const NavigationVector* velocity;
    const NavigationVector* displacement;
    const uint16_t*        stripe_count;

    const Quaternion<NavigationType>* orientation;
    NavigationType braking_dist;
    NavigationType em_braking_dist;
  };

  /**
   * @brief Construct a new Navigation object
   *
   * @param post_calibration_barrier Navigation module will wait on this barrier at the end of the
   *                                 transition to 'operational' state. It is primarily meant for
   *                                 syncing with motors module.
   */
  Navigation(Barrier& post_calibration_barrier,
             Logger& log = System::getLogger(),
             std::string file_path = "../BeagleBone_black/data/configuration/NavSettings.txt");

  /**
   * @brief Get the acceleration value
   *
   * @return NavigationType Returns the forward component of acceleration vector (negative when
   *                        decelerating)
   */
  NavigationType getAcceleration() const;
  /**
   * @brief Get the velocity value
   *
   * @return NavigationType Returns the forward component of velocity vector
   */
  NavigationType getVelocity() const;
  /**
   * @brief Get the displacement value
   *
   * @return NavigationType Returns the forward component of displacement vector
   */
  NavigationType getDisplacement() const;
  /**
   * @brief Get the emergency braking distance in metres
   *
   * @return NavigationType emergency braking distance in metres
   */
  NavigationType getEmergencyBrakingDistance() const;
  /**
   * @brief Get the braking distances in metres
   *
   * @return NavigationType braking distance in metres
   */
  NavigationType getBrakingDistance() const;
  /**
   * @brief Get the status of the nav module
   *
   * @return ModuleStatus Status of the nav module
   */
  ModuleStatus getStatus() const;
  /**
   * @brief Get the all nav data. Useful for debugging and analysis
   *
   * @return const FullOutput&
   */
  const FullOutput& getAll();
  /**
   * @brief Starts the calibration phase if the module's status is `kInit`.
   *
   * @return true  Navigation module is now in the calibration phase
   * @return false Calibration cannot be started
   */
  bool startCalibration();
  /**
   * @brief Transition the navigation module to an operational state so that it starts producing
   *        useful output. Hits the `post_calibration_barrier` before returning `true` (to indicate
   *        to motors that the calibration is done).
   * @return true  Transition has been successful
   * @return false Transition is not possible at the moment
   */
  bool finishCalibration();
  /**
   * @brief Initializes filters and transitions nav from kStart to kInit
   *
   * @param sc        Sensor variance data
   * @param readings  Latest sensor readings
   */
  void init(SensorCalibration sc, Sensors readings);
  void update(Input);

 private:
  /**
   * @brief Front-right, rear-right, rear-left, and front-left proxi sensor distances. This is
   *        likely temporary and will be replaced by an array or other suitable data structure once
   *        the algorithm using it is complete.
   */
#ifdef PROXI
  struct Proximities {
    float fr;  // mm
    float rr;  // mm
    float rl;  // mm
    float fl;  // mm
  };
#endif

  static constexpr int kMinNumCalibrationSamples = 200000;
  static const Settings kDefaultSettings;
  /**
   * @brief Calculates distance to the given stripe, the next stripe and the one after that.
   *
   * @param  stripe_count  Number of stripes.
   * @return std::array<NavigationType, 3> Index 0 contains distance to last stripe (should be
   *                                       negative); indices 1 and 2 contain distances to the
   *                                       next 2 stripes (both should be positive).
   */
  std::array<NavigationType, 3> getNearestStripeDists(uint16_t stripe_count);

  /**
   * @brief Updates navigation based on new IMU and stripe counter readings. Should be called when
   *        IMU, proximity sensors, and stripe counter have all been updated.
   *
   * @param imus         Datapoint of an array of IMU readings
   * @param[in] proxis   Array of proximity readings
   * @param scs          Array of stripe counter readings
   */

  void imuUpdate(DataPoint<ImuArray> imus);
#ifdef PROXI
  void proximityUpdate(ProximityArray proxis);
#endif
  void calibrationUpdate(ImuArray imus);
  void gyroUpdate(DataPoint<NavigationVector> angular_velocity);  // Point number 1
  void accelerometerUpdate(DataPoint<NavigationVector> acceleration);  // Points 3, 4, 5, 6
#ifdef PROXI
  void proximityOrientationUpdate(Proximities ground, Proximities rail);  // Point number 7
  void proximityDisplacementUpdate(Proximities ground, Proximities rail);  // Point number 7
#endif
  void stripeCounterUpdate(StripeCounterArray scs);  // Point number 7
  void opticalEncoderUpdate(array<float, Sensors::kNumOptEnc> optical_enc_distance);
  void readDataFromFile(std::string file_path);

  // Admin stuff
  Barrier& post_calibration_barrier_;
  Logger& log_;
  Settings settings_;
  ModuleStatus status_;
  FullOutput out_;

  // Calibration variables
  bool is_calibrating_;
  int num_gravity_samples_;
  std::array<NavigationVector, Sensors::kNumImus> g_;  // Acc offsets (gravitational acc)
  int num_gyro_samples_;
  std::array<NavigationVector, Sensors::kNumImus> gyro_offsets_;  // Measured during calibration

  // Most up-to-date values of pod's acceleration, velocity and displacement in 3D; used for output
  NavigationVector acceleration_;
  DataPoint<NavigationVector> velocity_;
  DataPoint<NavigationVector> displacement_;
  uint16_t stripe_count_;

  // Internal data that is not published
  DataPoint<NavigationVector> prev_angular_velocity_;  // To calculate how much has the pod rotated
  Quaternion<NavigationType> orientation_;  // Pod's orientation is updated with every gyro reading

  // Filters for reducing noise in sensor data before processing the data in any other way
  std::array<Kalman<NavigationVector>, Sensors::kNumImus> acceleration_filter_;  // One for each IMU
  std::array<Kalman<NavigationVector>, Sensors::kNumImus> gyro_filter_;          // One for each IMU
#ifdef PROXI
  std::array<Kalman<float>, 2*Sensors::kNumProximities> proximity_filter_;
#endif
  Integrator<NavigationVector> acceleration_integrator_;  // Acceleration to velocity
  Integrator<NavigationVector> velocity_integrator_;      // Velocity to displacement
  Differentiator<NavigationType> stripe_differentiator_;  // Stripe cnt distance to velocity
#ifdef PROXI
  Differentiator<Vector<NavigationType, 2>> proxi_differentiator_;
#endif
};

}}  // namespace hyped::navigation

#endif  // BEAGLEBONE_BLACK_NAVIGATION_NAVIGATION_HPP_

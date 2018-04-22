/*
 * Organisation: HYPED
 * Date: 18/02/2018
 * Description: Class for data exchange between sub-team threads and structures
 * for holding data produced by each of the sub-teams.
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may
 *    not use this file except in compliance with the License. You may obtain a
 *    copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *    WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 *    License for the specific language governing permissions and limitations
 *    under the License.
 */

#ifndef BEAGLEBONE_BLACK_DATA_DATA_HPP_
#define BEAGLEBONE_BLACK_DATA_DATA_HPP_

#include <array>
#include <cstdint>

#include "data/data_point.hpp"
#include "utils/math/vector.hpp"
#include "utils/concurrent/lock.hpp"

namespace hyped {

// imports
using utils::concurrent::Lock;
using utils::math::Vector;

namespace data {

// -----------------------------------------------------------------------------
// State Machine
// -----------------------------------------------------------------------------
enum State {
  kIdle,
  kAccelerating,
  kDecelerating,
  kEmergencyBraking,
  kRunComplete,
  kFailureStopped,
  kExiting,
  kFinished
};

struct StateMachine {
  bool critical_failure;
  State current_state;
};

// -----------------------------------------------------------------------------
// Navigation
// -----------------------------------------------------------------------------
typedef float NavigationType;
struct Navigation {
  NavigationType distance;
  NavigationType velocity;
  NavigationType acceleration;
};

// -----------------------------------------------------------------------------
// Raw Sensor data
// -----------------------------------------------------------------------------

struct Imu {
  DataPoint<Vector<NavigationType, 3> > acc;
  DataPoint<Vector<NavigationType, 3> > gyr;
};

struct Proximity {
  uint8_t val;
};

struct Battery {
  Vector<int16_t, 7> temperatures;
  uint8_t voltage;
};

/*struct StripeCount {
  DataPoint<uint32_t> count;
};//*/
typedef DataPoint<uint32_t> StripeCount;

struct Sensors {
  static constexpr int kNumImus = 8;
  static constexpr int kNumProximities = 24;

  std::array<Imu, kNumImus> imu;
  std::array<Proximity, kNumProximities> proxy;
  StripeCount stripe_cnt;
};

struct Batteries {
  static constexpr int kNumLPBatteries = 2;
  static constexpr int kNumHPBatteries = 2;

  std::array<Battery, kNumLPBatteries> low_power_batteries;
  std::array<Battery, kNumHPBatteries> high_power_batteries;
};

// -----------------------------------------------------------------------------
// Motor data
// -----------------------------------------------------------------------------

enum MotorState {
  kCriticalFailure,
  kMotorIdle,
  kMotorAccelerating,
  kMotorDecelerating,
  kMotorStopping,
  kMotorStopped
};

struct Motors {
  MotorState current_motor_state;
  int32_t rpm_FL;
  int32_t rpm_FR;
  int32_t rpm_BL;
  int32_t rpm_BR;
};

// -----------------------------------------------------------------------------
// Communications data
// -----------------------------------------------------------------------------

struct Communications {
  bool stopCommand;
  bool killPowerCommand;
};

// -----------------------------------------------------------------------------
// Common Data structure/class
// -----------------------------------------------------------------------------
/**
 * @brief      A singleton class managing the data exchange between sub-team
 * threads.
 */
class Data {
 public:
  /**
   * @brief      Always returns a reference to the only instance of `Data`.
   */
  static Data& getInstance();

  /**
   * @brief      Retrieves data related to the state machine. Data has high priority.
   */
  StateMachine getStateMachineData();

  /**
   * @brief      Should be called by state machine team to update data.
   */
  void setStateMachineData(const StateMachine& sm_data);

  /**
   * @brief      Retrieves data produced by navigation sub-team.
   */
  Navigation getNavigationData();

  /**
   * @brief      Should be called by navigation sub-team whenever they have new data.
   */
  void setNavigationData(const Navigation& nav_data);

  /**
   * @brief      Retrieves data from all sensors
   */
  Sensors getSensorsData();

  /**
   * @brief      Should be called to update sensor data.
   */
  void setSensorsData(const Sensors& sensors_data);

    /**
   * @brief      Retrieves data from the batteries.
   */
  Sensors getBatteryData();

  /**
   * @brief      Should be called to update battery data
   */
  void setBatteryData(const Batteries& batteries_data);

  /**
   * @brief      Retrieves data produced by each of the four motors.
   */
  Motors getMotorData();

  /**
   * @brief      Should be called to update motor data.
   */
  void setMotorData(const Motors& motor_data);

  /**
     * @brief      Retrieves data on whether stop/kill power commands have been issued.
     */
    Communications getCommunicationsData();

    /**
     * @brief      Should be called to update motor data.
     */
    void setCommunicationsData(const Communications& communications_data);

 private:
  StateMachine state_machine_;
  Navigation navigation_;
  Sensors sensors_;
  Motors motors_;
  Batteries batteries_;
  Communications communications_;

  // locks for data substructures
  Lock lock_state_machine_;
  Lock lock_navigation_;
  Lock lock_sensors_;
  Lock lock_motors_;
  Lock lock_communications_;
  Lock lock_batteries_;
};

}}  // namespace data::hyped

#endif  // BEAGLEBONE_BLACK_DATA_DATA_HPP_

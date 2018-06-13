/*
 * Organisation: HYPED
 * Date: 18/02/2018
 * Description: Class for data exchange between sub-team threads and structures
 * for holding data produced by each of the sub-teams.
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

#ifndef BEAGLEBONE_BLACK_DATA_DATA_HPP_
#define BEAGLEBONE_BLACK_DATA_DATA_HPP_

#include <array>
#include <cstdint>

#include "data/data_point.hpp"
#include "utils/math/vector.hpp"
#include "utils/concurrent/lock.hpp"

using std::array;

namespace hyped {

// imports
using utils::concurrent::Lock;
using utils::math::Vector;

namespace data {

// -------------------------------------------------------------------------------------------------
// Global Module States
// -------------------------------------------------------------------------------------------------

enum class ModuleStatus {
  kStart,   // Initial module state
  kInit,  // SM transistions to Calibrating if all modules have Init status.
  kReady,  // SM transistions to Ready if Motors and Navigation have the Ready status.
  kCriticalFailure  // SM transitions to EmergencyBraking/FailureStopped
};



// -------------------------------------------------------------------------------------------------
// State Machine States
// -------------------------------------------------------------------------------------------------
enum State {
  kIdle,
  kCalibrating,
  kReady,
  kAccelerating,
  kDecelerating,
  kEmergencyBraking,
  kRunComplete,
  kFailureStopped,
  kExiting,
  kFinished,
  kInvalid,
  num_states
};

extern const char* states[num_states];

struct StateMachine {
  bool critical_failure;
  State current_state;
};

// -------------------------------------------------------------------------------------------------
// Navigation
// -------------------------------------------------------------------------------------------------
enum class NavigationState {
  kCalibrating,      ///< Navigation module is calibrating. Pod must not be moved in this state.
  kCriticalFailure,  ///< Navigation module has problems and cannot provide reliable output
  kOperational,      ///< Navigation module is working fine and providing reliable output
  kReady             ///< Navigation module is still calibrating but ready to transition to
                     ///  `kOperational` state
};

typedef float NavigationType;
struct Navigation {
  NavigationState state;
  NavigationType  distance;
  NavigationType  velocity;
  NavigationType  acceleration;
  NavigationType  emergency_braking_distance;
  ModuleStatus module_status;
};

// -------------------------------------------------------------------------------------------------
// Raw Sensor data
// -------------------------------------------------------------------------------------------------
struct Sensor {
  bool operational;
};

typedef Vector<NavigationType, 3> NavigationVector;
struct Imu : public Sensor {
  DataPoint<NavigationVector> acc;
  DataPoint<NavigationVector> gyr;
};

struct Proximity : public Sensor {
  uint8_t val;
};

struct StripeCounter : public Sensor {
  DataPoint<uint32_t> count;
};

struct Sensors {
  ModuleStatus module_status;
  static constexpr int kNumImus = 8;
  static constexpr int kNumProximities = 24;

  array<Imu, kNumImus> imu;
  array<Proximity, kNumProximities> proxi;
  StripeCounter stripe_counter;
};

struct Battery {
  uint16_t  voltage;
  int8_t    temperature;
};

struct Batteries {
  ModuleStatus module_status;
  static constexpr int kNumLPBatteries = 2;
  static constexpr int kNumHPBatteries = 2;

  array<Battery, kNumLPBatteries> low_power_batteries;
  array<Battery, kNumHPBatteries> high_power_batteries;
};

// -------------------------------------------------------------------------------------------------
// Motor data
// -------------------------------------------------------------------------------------------------

enum MotorState {
  kCriticalFailure,
  kPreOperational,
  kMotorAccelerating,
  kMotorDecelerating,
  kMotorStopping,
  kMotorStopped
};

struct Motors {
  // ModuleStatus module_status;  // @TODO (Sean) Uncomment and integrate
  MotorState current_motor_state;
  int32_t motor_velocity_1;
  int32_t motor_velocity_2;
  int32_t motor_velocity_3;
  int32_t motor_velocity_4;
  int16_t motor_torque_1;
  int16_t motor_torque_2;
  int16_t motor_torque_3;
  int16_t motor_torque_4;
};

// -------------------------------------------------------------------------------------------------
// Communications data
// -------------------------------------------------------------------------------------------------

struct Communications {
  ModuleStatus module_status;
  bool stopCommand;   // deprecate
  bool launchCommand;
  bool resetCommand;
  float run_length;
};

// -------------------------------------------------------------------------------------------------
// Common Data structure/class
// -------------------------------------------------------------------------------------------------
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
   * @brief       Retrieves only StripeCount part from Sensors data
   */
  StripeCounter getStripeCounterData();

  /**
   * @brief       Should be called to update StripeCount part in Sensors data
   */
  void setStripeCounterData(const StripeCounter& stripe_counter);

  /**
   * @brief      Retrieves data from the batteries.
   */
  Batteries getBatteriesData();

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
   * @brief      Should be called to update communications data.
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

/*
 * Author: Sean Mullan
 * Organisation: HYPED
 * Date: 10/07/18
 * Description: Interface for controllers to enable use of real or fake controllers
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#ifndef BEAGLEBONE_BLACK_SENSORS_CONTROLLER_INTERFACE_HPP_
#define BEAGLEBONE_BLACK_SENSORS_CONTROLLER_INTERFACE_HPP_

namespace hyped {
namespace motor_control {

enum ControllerState {
  kNotReadyToSwitchOn,
  kSwitchOnDisabled,
  kReadyToSwitchOn,
  kSwitchedOn,
  kOperationEnabled,
  kQuickStopActive,
  kFaultReactionActive,
  kFault,
};

class ControllerInterface {
  public:
    virtual void registerController() = 0;
    virtual void configure() = 0;
    virtual void enterOperational() = 0;
    virtual void enterPreOperational() = 0;
    virtual void checkState() = 0;
    virtual void sendTargetVelocity(int32_t target_velocity) = 0;
    virtual void updateActualVelocity() = 0;
    virtual int32_t getVelocity() = 0;
    virtual void quickStop() = 0;
    virtual void healthCheck() = 0;
    virtual bool getFailure() = 0;
    virtual ControllerState getControllerState() = 0;
};

}}  // namespace hyped::motor_control

#endif  // BEAGLEBONE_BLACK_SENSORS_CONTROLLER_INTERFACE_HPP_
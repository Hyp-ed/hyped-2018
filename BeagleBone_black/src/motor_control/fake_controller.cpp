/*
 * Author: Sean Mullan and Jack Horsburgh
 * Organisation: HYPED
 * Date: 5/05/18
 * Description:
 * A controller object represents one motor controller. Object is used to request data specific to
 * the controller.
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

#include "motor_control/fake_controller.hpp"
#include <cstdint>

#include "utils/logger.hpp"
#include "data/data.hpp"
#include "utils/io/can.hpp"

namespace hyped {
namespace motor_control {

using utils::io::can::Frame;
using utils::concurrent::Thread;
using utils::Timer;

FakeController::FakeController(Logger& log, uint8_t id)
  : log_(log),
    data_(data::Data::getInstance()),
    motor_data_(data_.getMotorData()),
    node_id_(id),
    critical_failure_(false),
    actual_velocity_(0),
    actual_torque_(0),
    started_(false)
{
}

void FakeController::registerController()
{}

void FakeController::configure()
{
  log_.INFO("MOTOR", "Controller %d: Configuring...", node_id_);
  if(node_id_ == 1) {
    RPMvTime.open("RPMvTime");
    if(!RPMvTime.is_open()) {
      log_.ERR("MOTOR", "Could not open RPM v Time text file");
    } else {
      RPMvTime << "Time\t" << "RPM\n";
    }
  }
}

void FakeController::enterOperational()
{
  state_ = kOperationEnabled;
  log_.DBG1("MOTOR", "Controller %d: Entering Operational");
}

void FakeController::enterPreOperational()
{
  state_ = kSwitchOnDisabled;
  log_.DBG1("MOTOR", "Controller %d: Shutting down motor", node_id_);
}

void FakeController::checkState()
{
  log_.DBG1("MOTOR", "Controller %d: Checking status", node_id_);
}

void FakeController::sendTargetVelocity(int32_t target_velocity)
{
  if (!started_ && node_id_ == 1) {
    timer.start();
    started_ = true;
  }
  log_.DBG2("MOTOR", "Controller %d: Updating target velocity to %d", node_id_, target_velocity);
  actual_velocity_ = target_velocity;
  if (node_id_ == 1) {
    RPMvTime << (timer.getTimeMicros() / 1000) << "\t" << actual_velocity_;
  }
}

void FakeController::updateActualVelocity()
{}

void updateActualTorque()
{}

int32_t FakeController::getVelocity()
{
  return actual_velocity_;
}

void FakeController::quickStop()
{
  log_.DBG1("MOTOR", "Controller %d: Sending quickStop command", node_id_);
}

void FakeController::healthCheck()
{}

bool FakeController::getFailure()
{
  return critical_failure_;
}

ControllerState FakeController::getControllerState()
{
  return state_;
}

}}  // namespace hyped::motor_control
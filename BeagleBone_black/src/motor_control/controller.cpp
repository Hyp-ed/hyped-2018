/*
 * Author: Sean Mullan
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

#include "motor_control/controller.hpp"
#include <cstdint>

#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/io/can.hpp"

namespace hyped {
namespace motor_control {

using hyped::utils::io::can::Frame;

/* SDO CAN Frames (Little Edian) composed as follows:
 * CAN ID:    COB-ID = Function + Node ID,
 * Len:       Size of object dictionary entry
 * Byte 0:    Command
 * Bytes 1-2: Object Dictionary Index
 * Byte  3:   Object Dictionary Sub-Index
 * Bytes 4-7: Data
 */

/* NMT CAN Frames composed as follows:
 * CAN ID: 0x000
 * Len:    2
 * Byte 1: NMT Function
 * Byte 2: Target Node
 */

constexpr uint32_t kSDO_RECEIVE            = 0x600;
constexpr uint8_t  kREAD_OBJECT            = 0x40;
constexpr uint8_t  kWRITE_1_BYTE           = 0x2F;
constexpr uint8_t  kWRITE_2_BYTES          = 0x2B;
constexpr uint8_t  kWRITE_3_BYTES          = 0x27;
constexpr uint8_t  kWRITE_4_BYTES          = 0x23;
constexpr uint32_t kNMT_RECEIVE            = 0x000;
constexpr uint8_t  kNMT_OPERATIONAL        = 0x01;
constexpr uint8_t  kNMT_STOP               = 0x02;
constexpr uint8_t  kNMT_PREOPERATIONAL     = 0x80;
constexpr uint8_t  kNMT_RESET_NODE         = 0x81;
constexpr uint8_t  kNMT_RESET_COMMS        = 0x82;

// Status masks, used to mask the return result of a CAN message
// pg51 CANOpen_Motion_Control.pdf
constexpr uint16_t kNotReadyToSwitch       = 0x0000;
constexpr uint16_t kSwitchOnDisabled       = 0x0040;
constexpr uint16_t kReadyToSwitchOn        = 0x0021;
constexpr uint16_t kSwitchedOn             = 0x0023;
constexpr uint16_t kOperationEnabled       = 0x0027;
constexpr uint16_t kQuickStopActive        = 0x0007;
constexpr uint16_t kFaultReactionActive    = 0x0008;
// Unsure about this check datasheet TODO(anyone)
constexpr uint16_t kFault                  = 0x0008;

// Masks for profile velocity status
// pg123 'CANOpen_Motion_Control.pdf
constexpr uint16_t kTargetReached          = 0x0400;
constexpr uint16_t kSpeedEqualToZero       = 0x1000;
constexpr uint16_t kMaxSlippageReached     = 0x2000;

Controller::Controller(Logger& log, uint8_t id)
  : log_(log),
    can_(Can::getInstance()),
    node_id_(id),
    critical_failure_(false),
    error_(false)
{}

void Controller::registerController()
{
  can_.registerController(this);
}

void Controller::configure()
{
  log_.INFO("MOTOR", "Controller : Configuring...");

  // Set motor pole pairs to 10
  utils::io::can::Frame message;
  message.id        = kSDO_RECEIVE + node_id_;
  message.extended  = false;
  message.len       = 8;
  message.data[0]   = kWRITE_1_BYTE;
  message.data[1]   = 0x33;
  message.data[2]   = 0x20;
  message.data[3]   = 0x00;
  message.data[4]   = 0x0A;
  message.data[5]   = 0x00;
  message.data[6]   = 0x00;
  message.data[7]   = 0x00;

  can_.send(message);
  log_.DBG1("MOTOR", "Controller : Configuring motor poles");

  // Set feedback type to SSI
  message.id        = kSDO_RECEIVE + node_id_;
  message.extended  = false;
  message.len       = 8;
  message.data[0]   = kWRITE_1_BYTE;
  message.data[1]   = 0x40;
  message.data[2]   = 0x20;
  message.data[3]   = 0x01;
  message.data[4]   = 0x02;
  message.data[5]   = 0x00;
  message.data[6]   = 0x00;
  message.data[7]   = 0x00;

  can_.send(message);
  log_.DBG1("MOTOR", "Controller : Configuring feedback type");

  // Set motor phase offset compensation to 190
  message.id        = kSDO_RECEIVE + node_id_;
  message.extended  = false;
  message.len       = 8;
  message.data[0]   = kWRITE_2_BYTES;
  message.data[1]   = 0x40;
  message.data[2]   = 0x20;
  message.data[3]   = 0x08;
  message.data[4]   = 0xBE;
  message.data[5]   = 0x00;
  message.data[6]   = 0x00;
  message.data[7]   = 0x00;

  can_.send(message);
  log_.DBG1("MOTOR", "Controller : Configuring motor phase offset");

  // Set over voltage limit to 125
  message.id        = kSDO_RECEIVE + node_id_;
  message.extended  = false;
  message.len       = 8;
  message.data[0]   = kWRITE_2_BYTES;
  message.data[1]   = 0x54;
  message.data[2]   = 0x20;
  message.data[3]   = 0x00;
  message.data[4]   = 0x7D;
  message.data[5]   = 0x00;
  message.data[6]   = 0x00;
  message.data[7]   = 0x00;

  can_.send(message);
  log_.DBG1("MOTOR", "Controller : Configuring over voltage limit");

  // Set under voltage minimum to 30
  message.id        = kSDO_RECEIVE + node_id_;
  message.extended  = false;
  message.len       = 8;
  message.data[0]   = kWRITE_2_BYTES;
  message.data[1]   = 0x55;
  message.data[2]   = 0x20;
  message.data[3]   = 0x03;
  message.data[4]   = 0x1E;
  message.data[5]   = 0x00;
  message.data[6]   = 0x00;
  message.data[7]   = 0x00;

  can_.send(message);
  log_.DBG1("MOTOR", "Controller : Configuring under voltage limit");

  // Set motor rated current to 80,000 mA
  message.id        = kSDO_RECEIVE + node_id_;
  message.extended  = false;
  message.len       = 8;
  message.data[0]   = kWRITE_4_BYTES;
  message.data[1]   = 0x75;
  message.data[2]   = 0x60;
  message.data[3]   = 0x00;
  message.data[4]   = 0x80;
  message.data[5]   = 0x38;
  message.data[6]   = 0x01;
  message.data[7]   = 0x00;

  can_.send(message);
  log_.DBG1("MOTOR", "Controller : Configuring motor rated current");

  // Set motor rated torque to 80N
  message.id        = kSDO_RECEIVE + node_id_;
  message.extended  = false;
  message.len       = 8;
  message.data[0]   = kWRITE_4_BYTES;
  message.data[1]   = 0x76;
  message.data[2]   = 0x60;
  message.data[3]   = 0x00;
  message.data[4]   = 0x80;
  message.data[5]   = 0x38;
  message.data[6]   = 0x01;
  message.data[7]   = 0x00;

  can_.send(message);
  log_.DBG1("MOTOR", "Controller : Configuring motor rated torque");

  // Reset node after configuration
  message.id        = kNMT_RECEIVE;
  message.extended  = false;
  message.len       = 2;
  message.data[0]   = kNMT_RESET_NODE;
  message.data[1]   = node_id_;

  can_.send(message);
  log_.DBG1("MOTOR", "Controller : Reset node");
}

void Controller::enterOperational()
{
  utils::io::can::Frame message;
  // Check for warning messages
  message.id        = kSDO_RECEIVE + node_id_;
  message.extended  = false;
  message.len       = 8;
  message.data[0]   = kREAD_OBJECT;
  message.data[1]   = 0x27;
  message.data[2]   = 0x20;
  message.data[3]   = 0x00;
  message.data[4]   = 0x00;
  message.data[5]   = 0x00;
  message.data[6]   = 0x00;
  message.data[7]   = 0x00;

  can_.send(message);
  log_.INFO("MOTOR", "Controller : Checking for warnings");

  // Check for error messages
  message.id        = kSDO_RECEIVE + node_id_;
  message.extended  = false;
  message.len       = 8;
  message.data[0]   = kREAD_OBJECT;
  message.data[1]   = 0x3F;
  message.data[2]   = 0x60;
  message.data[3]   = 0x00;
  message.data[4]   = 0x00;
  message.data[5]   = 0x00;
  message.data[6]   = 0x00;
  message.data[7]   = 0x00;

  can_.send(message);
  log_.INFO("MOTOR", "Controller : Checking for errors");

  while (!error_status_checked_);

  if (!error_) {
    // Enter NMT Operational
    message.id        = kNMT_RECEIVE;
    message.extended  = false;
    message.len       = 2;
    message.data[0]   = kNMT_OPERATIONAL;
    message.data[1]   = node_id_;

    can_.send(message);
    log_.INFO("MOTOR", "Controller : NMT Operational");

    // Send shutdown message to tranition state (Disable drive function)
    message.id        = kSDO_RECEIVE + node_id_;
    message.extended  = false;
    message.len       = 8;
    message.data[0]   = kWRITE_2_BYTES;
    message.data[1]   = 0x40;
    message.data[2]   = 0x60;
    message.data[3]   = 0x06;
    message.data[4]   = 0x00;
    message.data[5]   = 0x00;
    message.data[6]   = 0x00;
    message.data[7]   = 0x00;

    can_.send(message);
    log_.DBG1("MOTOR", "Controller : Disabling drive function");

    // Send switch on message (Disable breaks)
    message.id        = kSDO_RECEIVE + node_id_;
    message.extended  = false;
    message.len       = 8;
    message.data[0]   = kWRITE_2_BYTES;
    message.data[1]   = 0x40;
    message.data[2]   = 0x60;
    message.data[3]   = 0x07;
    message.data[4]   = 0x00;
    message.data[5]   = 0x00;
    message.data[6]   = 0x00;
    message.data[7]   = 0x00;

    can_.send(message);
    log_.DBG1("MOTOR", "Controller : Disabling breaks");

    // Send enter operational message
    message.id        = kSDO_RECEIVE + node_id_;
    message.extended  = false;
    message.len       = 8;
    message.data[0]   = kWRITE_2_BYTES;
    message.data[1]   = 0x40;
    message.data[2]   = 0x60;
    message.data[3]   = 0x0F;
    message.data[4]   = 0x00;
    message.data[5]   = 0x00;
    message.data[6]   = 0x00;
    message.data[7]   = 0x00;

    can_.send(message);
    log_.DBG1("MOTOR", "Controller : Enabling drive function");

    // Enable velocity mode
    message.id        = kSDO_RECEIVE + node_id_;
    message.extended  = false;
    message.len       = 8;
    message.data[0]   = kWRITE_2_BYTES;
    message.data[1]   = 0x40;
    message.data[2]   = 0x60;
    message.data[3]   = 0x03;
    message.data[4]   = 0x00;
    message.data[5]   = 0x00;
    message.data[6]   = 0x00;
    message.data[7]   = 0x00;

    can_.send(message);
    log_.DBG1("MOTOR", "Controller : Enabling velocity mode");
  } else {
    log_.INFO("MOTOR", "Controller : ERROR");  // TODO(Anyone): Process error message
  }
}

void Controller::checkStatus()
{
  // Check Statusword in object dictionary
  utils::io::can::Frame message;
  message.id        = kSDO_RECEIVE + node_id_;
  message.extended  = false;
  message.len       = 8;
  message.data[0]   = kREAD_OBJECT;
  message.data[1]   = 0x41;
  message.data[2]   = 0x60;
  message.data[3]   = 0x00;
  message.data[4]   = 0x00;
  message.data[5]   = 0x00;
  message.data[6]   = 0x00;
  message.data[7]   = 0x00;

  can_.send(message);
  log_.DBG1("MOTOR", "Controller : Checking status");
}
void Controller::sendTargetVelocity(int32_t target_velocity)
{/*EMPTY*/}

void Controller::sendTargetTorque(int16_t target_torque)
{/*EMPTY*/}

int32_t Controller::requestActualVelocity()
{
  return 0;
}

int16_t Controller::requestActualTorque()
{
  return 0;
}

void Controller::processNewData(utils::io::can::Frame& message)
{/*EMPTY*/}

void Controller::quickStop()
{/*EMPTY*/}

bool Controller::getFailure()
{
  return critical_failure_;
}

uint8_t Controller::getId()
{
  return node_id_;
}

}}  // namespace hyped::motor_control

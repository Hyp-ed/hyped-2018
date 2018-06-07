/*
 * Author: Sean Mullan and Jack Horsburgh
 * Organisation: HYPED
 * Date: 5/05/18
 * Description:
 * A controller object represents one motor controller. Object is used to request data specific to
 * the controller.
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License", node_id_); you may not use this file
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

using utils::io::can::Frame;
using utils::concurrent::Thread;

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

// Types of CANopen messages, these are used for CAN ID's
constexpr uint32_t kEMGY_TRANSMIT          = 0x80;
constexpr uint32_t kSDO_RECEIVE            = 0x600;
constexpr uint32_t kSDO_TRANSMIT           = 0x580;
constexpr uint32_t kNMT_RECEIVE            = 0x000;
constexpr uint32_t kNMT_TRANSMIT           = 0x700;

// Function codes for sending messages
constexpr uint8_t  kREAD_OBJECT            = 0x40;
constexpr uint8_t  kWRITE_1_BYTE           = 0x2F;
constexpr uint8_t  kWRITE_2_BYTES          = 0x2B;
constexpr uint8_t  kWRITE_3_BYTES          = 0x27;
constexpr uint8_t  kWRITE_4_BYTES          = 0x23;

// Network management commands
constexpr uint8_t  kNMT_OPERATIONAL        = 0x01;
constexpr uint8_t  kNMT_STOP               = 0x02;
constexpr uint8_t  kNMT_PREOPERATIONAL     = 0x80;
constexpr uint8_t  kNMT_RESET_NODE         = 0x81;
constexpr uint8_t  kNMT_RESET_COMMS        = 0x82;

Controller::Controller(Logger& log, uint8_t id)
  : log_(log),
    can_(Can::getInstance()),
    node_id_(id),
    critical_failure_(false),
    actual_velocity_(0),
    actual_torque_(0),
    state_(kNotReadyToSwitchOn)
{
  SDOMessage.id       = kSDO_RECEIVE + node_id_;
  SDOMessage.extended = false;
  SDOMessage.len      = 8;
  NMTMessage.id       = kNMT_RECEIVE;
  NMTMessage.extended = false;
  NMTMessage.len      = 2;
}

void Controller::registerController()
{
  can_.registerController(this);
}

void Controller::configure()
{
  log_.INFO("MOTOR", "Controller %d: Configuring...", node_id_);

  // Set motor pole pairs to 10
  SDOMessage.data[0]   = kWRITE_1_BYTE;
  SDOMessage.data[1]   = 0x33;
  SDOMessage.data[2]   = 0x20;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x0A;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Configuring motor poles", node_id_);

  // Set feedback type to SSI
  SDOMessage.data[0]   = kWRITE_1_BYTE;
  SDOMessage.data[1]   = 0x40;
  SDOMessage.data[2]   = 0x20;
  SDOMessage.data[3]   = 0x01;
  SDOMessage.data[4]   = 0x02;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Configuring feedback type", node_id_);

  // Set motor phase offset compensation to 190
  SDOMessage.data[0]   = kWRITE_2_BYTES;
  SDOMessage.data[1]   = 0x40;
  SDOMessage.data[2]   = 0x20;
  SDOMessage.data[3]   = 0x08;
  SDOMessage.data[4]   = 0xBE;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Configuring motor phase offset", node_id_);

  // Set over voltage limit to 125
  SDOMessage.data[0]   = kWRITE_2_BYTES;
  SDOMessage.data[1]   = 0x54;
  SDOMessage.data[2]   = 0x20;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x7D;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Configuring over voltage limit", node_id_);

  // Set under voltage minimum to 25
  SDOMessage.data[0]   = kWRITE_2_BYTES;
  SDOMessage.data[1]   = 0x55;
  SDOMessage.data[2]   = 0x20;
  SDOMessage.data[3]   = 0x03;
  SDOMessage.data[4]   = 0x19;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Configuring under voltage limit", node_id_);

  // Set motor temperature sensor
  SDOMessage.data[0]   = kWRITE_1_BYTE;
  SDOMessage.data[1]   = 0x57;
  SDOMessage.data[2]   = 0x20;
  SDOMessage.data[3]   = 0x01;
  SDOMessage.data[4]   = 0x03;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Configuring motor temperature sensor", node_id_);

  // Set motor rated current to 80,000 mA
  SDOMessage.data[0]   = kWRITE_4_BYTES;
  SDOMessage.data[1]   = 0x75;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x80;
  SDOMessage.data[5]   = 0x38;
  SDOMessage.data[6]   = 0x01;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Configuring motor rated current", node_id_);

  // Set motor rated torque to 80N
  SDOMessage.data[0]   = kWRITE_4_BYTES;
  SDOMessage.data[1]   = 0x76;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x80;
  SDOMessage.data[5]   = 0x38;
  SDOMessage.data[6]   = 0x01;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Configuring motor rated torque", node_id_);
}

void Controller::enterOperational()
{
  // Enter NMT Operational
  SDOMessage.data[0]   = kNMT_OPERATIONAL;
  SDOMessage.data[1]   = node_id_;

  can_.send(NMTMessage);
  log_.INFO("MOTOR", "Controller %d: NMT Operational command sent", node_id_);

  // Enable velocity mode
  SDOMessage.data[0]   = kWRITE_2_BYTES;
  SDOMessage.data[1]   = 0x40;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x09;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Enabling velocity mode", node_id_);

  // Set target velocity to 0;
  this->sendTargetVelocity(0);

  // Send shutdown message to tranition from state 1 (Not ready to switch on)
  // to state 2 (Switch on disabled)
  SDOMessage.data[0]   = kWRITE_2_BYTES;
  SDOMessage.data[1]   = 0x40;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x06;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Shutdown command sent", node_id_);

  // Send switch on message to transition from state 2 (Switch on disabled)
  // to state 3 (Ready to switch on)
  SDOMessage.data[0]   = kWRITE_2_BYTES;
  SDOMessage.data[1]   = 0x40;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x07;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Switch on command sent", node_id_);

  // Send enter operational message
  SDOMessage.data[0]   = kWRITE_2_BYTES;
  SDOMessage.data[1]   = 0x40;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x0F;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Enabling drive function", node_id_);

  // Check warning status
  SDOMessage.data[0]   = kREAD_OBJECT;
  SDOMessage.data[1]   = 0x27;
  SDOMessage.data[2]   = 0x20;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x00;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.INFO("MOTOR", "Controller %d: Checking for warnings", node_id_);

  // Check error status
  SDOMessage.data[0]   = kREAD_OBJECT;
  SDOMessage.data[1]   = 0x3F;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x00;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.INFO("MOTOR", "Controller %d: Checking for errors", node_id_);
}

void Controller::enterPreOperational()
{
  // Disable active motors if fault occurs in another motor
  if (!critical_failure_) {
    // Send shutdown command
    SDOMessage.data[0]   = kWRITE_2_BYTES;
    SDOMessage.data[1]   = 0x40;
    SDOMessage.data[2]   = 0x60;
    SDOMessage.data[3]   = 0x00;
    SDOMessage.data[4]   = 0x06;
    SDOMessage.data[5]   = 0x00;
    SDOMessage.data[6]   = 0x00;
    SDOMessage.data[7]   = 0x00;

    can_.send(SDOMessage);
    log_.DBG1("MOTOR", "Controller %d: Shutting down motor", node_id_);

    // Enter NMT Operational
    SDOMessage.data[0]   = kNMT_PREOPERATIONAL;
    SDOMessage.data[1]   = node_id_;

    can_.send(NMTMessage);
    log_.INFO("MOTOR", "Controller %d: NMT Pre-Operational command sent", node_id_);
  }
}

void Controller::checkStatus()
{
  // Check Statusword in object dictionary
  SDOMessage.data[0]   = kREAD_OBJECT;
  SDOMessage.data[1]   = 0x41;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x00;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Checking status", node_id_);
}

void Controller::sendTargetVelocity(int32_t target_velocity)
{
  // Send 32 bit integer in Little Edian bytes
  // TODO(Anyone) Cover negative velocity case to control direction
  SDOMessage.data[0]   = kWRITE_4_BYTES;
  SDOMessage.data[1]   = 0xFF;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = target_velocity & 0xFF;
  SDOMessage.data[5]   = (target_velocity >> 8) & 0xFF;
  SDOMessage.data[6]   = (target_velocity >> 16) & 0xFF;
  SDOMessage.data[7]   = (target_velocity >> 24) & 0xFF;
  can_.send(SDOMessage);
  log_.DBG2("MOTOR", "Controller %d: Updating target velocity", node_id_);
}

void Controller::sendTargetTorque(int16_t target_torque)
{
  // Send 32 bit integer in Little Edian bytes
  // TODO(Anyone) Cover negative torque case to control direction
  SDOMessage.data[0]   = kWRITE_4_BYTES;
  SDOMessage.data[1]   = 0x71;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = target_torque & 0xFF;
  SDOMessage.data[5]   = (target_torque >> 8) & 0xFF;
  SDOMessage.data[6]   = (target_torque >> 16) & 0xFF;
  SDOMessage.data[7]   = (target_torque >> 24) & 0xFF;

  can_.send(SDOMessage);
  log_.DBG2("MOTOR", "Controller %d: Updating target torque", node_id_);
}

void Controller::updateActualVelocity()
{
  // Check actual velocity in object dictionary
  SDOMessage.data[0]   = kREAD_OBJECT;
  SDOMessage.data[1]   = 0x6C;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x00;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG2("MOTOR", "Controller %d: Reading actual velocity", node_id_);
}

void Controller::updateActualTorque()
{
  // Check actual velocity in object dictionary
  SDOMessage.data[0]   = kREAD_OBJECT;
  SDOMessage.data[1]   = 0x77;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x00;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG2("MOTOR", "Controller %d: Reading actual torque", node_id_);
}

int32_t Controller::getVelocity()
{
  return actual_velocity_;
}

int16_t Controller::getTorque()
{
  return actual_torque_;
}

void Controller::processNewData(utils::io::can::Frame& message)
{
  uint32_t id = message.id;
  if (id == kEMGY_TRANSMIT + node_id_) {
    processEmergencyMessage(message);
  } else if (id == kSDO_TRANSMIT + node_id_) {
    processSDOMessage(message);
  } else if (id == kNMT_TRANSMIT + node_id_) {
    processNMTMessage(message);
  } else {
    log_.ERR("MOTOR", "Controller %d: CAN message not recognised", node_id_);
  }
}

void Controller::processEmergencyMessage(utils::io::can::Frame& message)
{
  critical_failure_ = true;
  log_.ERR("MOTOR", "Controller %d: CAN Emergency", node_id_);
  // TODO(Anyone) Process error message
  int8_t index_1   = message.data[1];
  int8_t index_2   = message.data[2];
  int8_t sub_index = message.data[3];

  if (index_2 == 0x00) {
    log_.ERR("MOTOR", "Controller %d: No emergency/error", node_id_);
  }

  if (index_2 == 0x10) {
    log_.ERR("MOTOR", "Controller %d: Generic error", node_id_);
  }

  if (index_2 == 0x20 || index_2 == 0x21) {
    log_.ERR("MOTOR", "Controller %d: Current error", node_id_);
  }

  if (index_2 == 0x22) {
    log_.ERR("MOTOR", "Controller %d: Internal Current error", node_id_);
  }

  if (index_2 == 0x23) {
    log_.ERR("MOTOR", "Controller %d: Current on device ouput side error", node_id_);
  }

  if (index_2 == 0x30) {
    log_.ERR("MOTOR", "Controller %d: Voltage error", node_id_);
  }

  if (index_2 == 0x31) {
    log_.ERR("MOTOR", "Controller %d: Mains voltage error", node_id_);
  }

  if (index_2 == 0x31 && index_1 >= 0x10 && index_1 <= 0x13) {
    log_.ERR("MOTOR", "Controller %d: Mains over-voltage error", node_id_);
  }

  if (index_2 == 0x31 && index_1 >= 0x20 && index_1 <= 0x23) {
    log_.ERR("MOTOR", "Controller %d: Mains under-voltage error", node_id_);
  }

  if (index_2 == 0x32) {
    log_.ERR("MOTOR", "Controller %d: DC link voltage", node_id_);
  }

  if (index_2 == 0x32 && index_1 >= 0x10 && index_1 <= 0x12) {
    log_.ERR("MOTOR", "Controller %d: DC link over-voltage", node_id_);
  }

  if (index_2 == 0x32 && index_1 >= 0x20 && index_1 <= 0x22) {
    log_.ERR("MOTOR", "Controller %d: DC link under-voltage", node_id_);
  }

  if (index_2 == 0x33) {
    log_.ERR("MOTOR", "Controller %d: Output voltage", node_id_);
  }

  if (index_2 == 0x33 && index_1 >= 0x10 && index_1 <= 0x13) {
    log_.ERR("MOTOR", "Controller %d: Output over-voltage", node_id_);
  }

  if (index_2 >= 0x40 && index_2 <=0x44) {
    log_.ERR("MOTOR", "Controller %d: Temperature error", node_id_);
  }

  if (index_2 >= 0x50 && index_2 <= 0x54) {
    log_.ERR("MOTOR", "Controller %d: Device hardware error", node_id_);
  }

  if (index_2 == 0x55) {
    log_.ERR("MOTOR", "Controller %d: Device storage error", node_id_);
  }

  if (index_2 >= 0x60 && index_2 <=0x63) {
    log_.ERR("MOTOR", "Controller %d: Device software error", node_id_);
  }

  if (index_2 == 0x70) {
    log_.ERR("MOTOR", "Controller %d: Additional modules error", node_id_);
  }

  if (index_2 == 0x71) {
    log_.ERR("MOTOR", "Controller %d: Power error", node_id_);
  }

  if (index_2 == 0x71 && index_1 >= 0x10 && index_1 <= 0x13) {
    log_.ERR("MOTOR", "Controller %d: Brake chopper error", node_id_);
  }

  if (index_2 == 0x71 && index_1 >= 0x20 && index_1 <= 0x23) {
    log_.ERR("MOTOR", "Controller %d: Motor error", node_id_);
  }

  if (index_2 == 0x72) {
    log_.ERR("MOTOR", "Controller %d: Measurement circuit error", node_id_);
  }

  if (index_2 == 0x73) {
    log_.ERR("MOTOR", "Controller %d: Sensor error", node_id_);
  }

  if (index_2 == 0x74) {
    log_.ERR("MOTOR", "Controller %d: Computation circuit error", node_id_);
  }

  if (index_2 == 0x75) {
    log_.ERR("MOTOR", "Controller %d: Communication error", node_id_);
  }

  if (index_2 == 0x76) {
    log_.ERR("MOTOR", "Controller %d: Data storage error", node_id_);
  }

  if (index_2 == 0x80) {
    log_.ERR("MOTOR", "Controller %d: Monitoring error", node_id_);
  }

  if (index_2 == 0x81) {
    log_.ERR("MOTOR", "Controller %d: Commmunication error", node_id_);
  }

  if (index_2 == 0x82) {
    log_.ERR("MOTOR", "Controller %d: Protocol error", node_id_);
  }

  if (index_2 == 0x83) {
    log_.ERR("MOTOR", "Controller %d: Torque control error", node_id_);
  }

  if (index_2 == 0x84) {
    log_.ERR("MOTOR", "Controller %d: Velocity speed controller error", node_id_);
  }

  if (index_2 == 0x85) {
    log_.ERR("MOTOR", "Controller %d: Position controller error", node_id_);
  }

  if (index_2 == 0x86) {
    log_.ERR("MOTOR", "Controller %d: Positioning controller error", node_id_);
  }

  if (index_2 == 0x87) {
    log_.ERR("MOTOR", "Controller %d: Sync controller error", node_id_);
  }

  if (index_2 == 0x88) {
    log_.ERR("MOTOR", "Controller %d: Winding controller error", node_id_);
  }

  if (index_2 == 0x89) {
    log_.ERR("MOTOR", "Controller %d: Process data error", node_id_);
  }

  if (index_2 == 0x8A) {
    log_.ERR("MOTOR", "Controller %d: Control error", node_id_);
  }

  if (index_2 == 0x90) {
    log_.ERR("MOTOR", "Controller %d: External error", node_id_);
  }

  if (index_2 == 0xF0 && index_1 == 0x01) {
    log_.ERR("MOTOR", "Controller %d: Deceleration error", node_id_);
  }

  if (index_2 == 0xF0 && index_1 == 0x02) {
    log_.ERR("MOTOR", "Controller %d: Sub-synchronous run error", node_id_);
  }

  if (index_2 == 0xF0 && index_1 == 0x03) {
    log_.ERR("MOTOR", "Controller %d: Stroke operation error", node_id_);
  }

  if (index_2 == 0xF0 && index_1 == 0x04) {
    log_.ERR("MOTOR", "Controller %d: Control error", node_id_);
  }

  if (index_2 == 0xFF) {
    log_.ERR("MOTOR", "Controller %d: Manufacturer error", node_id_);
  }

  log_.DBG1("MOTOR", "index 1: %d, index 2: %d", index_1, index_2);
}

void Controller::processSDOMessage(utils::io::can::Frame& message)
{
  int8_t index_1   = message.data[1];
  int8_t index_2   = message.data[2];
  int8_t sub_index = message.data[3];

  // Process actual velocity. TODO(Anyone) Cover negative velocity case
  if (index_1 == 0x6C && index_2 == 0x60) {
    actual_velocity_ = (message.data[7] << 24) | (message.data[6] << 16)
                      | (message.data[5] << 8) | message.data[4];
    return;
  }

  // Process actual torque. TODO(Anyone) Cover negative torque case
  if (index_1 == 0x77 && index_2 == 0x60) {
    actual_torque_   = (message.data[5] << 8) | message.data[4];
    return;
  }

  // Process warning message
  if (index_1 == 0x27 && index_2 == 0x20 && sub_index == 0x00) {
    if (message.data[4] != 0 && message.data[5] != 0) {
      critical_failure_ = true;
      uint8_t warning_message = message.data[4];
      switch (warning_message) {
        case 0x01:
          log_.ERR("MOTOR", "Controller %d warning: Controller temperature exceeded");
          break;
        case 0x02:
          log_.ERR("MOTOR", "Controller %d warning: Motor temperature exceeded");
          break;
        case 0x04:
          log_.ERR("MOTOR", "Controller %d warning: DC link under voltage");
          break;
        case 0x08:
          log_.ERR("MOTOR", "Controller %d warning: DC link over voltage");
          break;
        case 0x10:
          log_.ERR("MOTOR", "Controller %d warning: DC over current");
          break;
        case 0x20:
          log_.ERR("MOTOR", "Controller %d warning: Stall protection active");
          break;
        case 0x40:
          log_.ERR("MOTOR", "Controller %d warning: Max velocity exceeded");
          break;
        default:
          log_.ERR("MOTOR", "Controller %d warning: Warning code %d", node_id_, warning_message);
      }
    }
    return;
  }

  // Process error message
  if (index_1 == 0x3F && index_2 == 0x60 && sub_index == 0x00) {
    if (message.data[4] != 0 && message.data[5] != 0) {
      critical_failure_ = true;
      uint16_t error_message = (message.data[5] << 8) | message.data[4];
      switch (error_message) {
        case 0x1000:
          log_.ERR("MOTOR", "Controller %d error: Unspecified error", node_id_);
          break;
        case 0x2220:
          log_.ERR("MOTOR", "Controller %d error: Overcurrent error", node_id_);
          break;
        case 0x3210:
          log_.ERR("MOTOR", "Controller %d error: DC link over voltage", node_id_);
          break;
        case 0xFF01:
          log_.ERR("MOTOR", "Controller %d error: Phase A current measurement", node_id_);
          break;
        case 0xFF02:
          log_.ERR("MOTOR", "Controller %d error: Phase B current measurement", node_id_);
          break;
        case 0xFF03:
          log_.ERR("MOTOR", "Controller %d error: High side FET short circuit", node_id_);
          break;
        case 0xFF04:
          log_.ERR("MOTOR", "Controller %d error: Low side FET short circuit", node_id_);
          break;
        case 0xFF05:
          log_.ERR("MOTOR", "Controller %d error: Low side phase 1 short circuit", node_id_);
          break;
        case 0xFF06:
          log_.ERR("MOTOR", "Controller %d error: Low side phase 2 short circuit", node_id_);
          break;
        case 0xFF07:
          log_.ERR("MOTOR", "Controller %d error: Low side phase 3 short circuit", node_id_);
          break;
        case 0xFF08:
          log_.ERR("MOTOR", "Controller %d error: High side phase 1 short circuit", node_id_);
          break;
        case 0xFF09:
          log_.ERR("MOTOR", "Controller %d error: High side phase 2 short circuit", node_id_);
          break;
        case 0xFF0A:
          log_.ERR("MOTOR", "Controller %d error: High side phase 3 short circuit", node_id_);
          break;
        case 0xFF0B:
          log_.ERR("MOTOR", "Controller %d error: Motor Feedback", node_id_);
          break;
        case 0xFF0C:
          log_.ERR("MOTOR", "Controller %d error: DC link under voltage", node_id_);
          break;
        case 0xFF0D:
          log_.ERR("MOTOR", "Controller %d error: Pulse mode finished", node_id_);
          break;
        case 0xFF0E:
          log_.ERR("MOTOR", "Controller %d error: Application error", node_id_);
          break;
        case 0xFF0F:
          log_.ERR("MOTOR", "Controller %d error: STO error", node_id_);
          break;
        case 0xFF10:
          log_.ERR("MOTOR", "Controller %d error: Controller temperature exceeded", node_id_);
          break;
      }
    }
    return;
  }

  // Process configuration messages
  if (index_1 == 0x33 && index_2 == 0x20 && sub_index == 0x00) {
        log_.DBG1("MOTOR", "Controller %d: Motor poles configured", node_id_);
    return;
  }
  if (index_1 == 0x40 && index_2 == 0x20 && sub_index == 0x01) {
        log_.DBG1("MOTOR", "Controller %d: Feedback type configured", node_id_);
    return;
  }
  if (index_1 == 0x40 && index_2 == 0x20 && sub_index == 0x08) {
        log_.DBG1("MOTOR", "Controller %d: Motor phase offset configured", node_id_);
    return;
  }
  if (index_1 == 0x54 && index_2 == 0x20 && sub_index == 0x00) {
        log_.DBG1("MOTOR", "Controller %d: Over voltage limit configured", node_id_);
    return;
  }
  if (index_1 == 0x55 && index_2 == 0x20 && sub_index == 0x03) {
        log_.DBG1("MOTOR", "Controller %d: Under voltage limit configured", node_id_);
    return;
  }
  if (index_1 == 0x75 && index_2 == 0x60 && sub_index == 0x00) {
        log_.DBG1("MOTOR", "Controller %d: Motor rated current configured", node_id_);
    return;
  }
  if (index_1 == 0x76 && index_2 == 0x60 && sub_index == 0x00) {
    log_.DBG1("MOTOR", "Controller %d: Motor rated torque configured", node_id_);
    return;
  }

  // Controlword updates
  if (index_1 == 0x40 && index_2 == 0x60 && sub_index == 0x00) {
    log_.DBG1("MOTOR", "Controller %d: Control Word updated", node_id_);
    return;
  }

  /* Process Statusword checks
   * xxxx xxxx x0xx 0000: Not ready to switch on
   * xxxx xxxx x1xx 0000: Switch on disabled
   * xxxx xxxx x01x 0001: Ready to switch on
   * xxxx xxxx x01x 0011: Switched on
   * xxxx xxxx x01x 0111: Operation enabled
   * xxxx xxxx x00x 0111: Quick stop active
   * xxxx xxxx x0xx 1111: Fault reaction active
   * xxxx xxxx x0xx 1000: Fault
   */
  if (index_1 == 0x41 && index_2 == 0x60 && sub_index == 0x00) {
    uint8_t status = message.data[4];
    if ((status << 4) == 0) {
      if (status & (1 << 6)) {
        state_ = kSwitchOnDisabled;
        log_.DBG1("MOTOR", "Controller %d state: Switch on disabled", node_id_);
        return;
      } else {
        state_ = kNotReadyToSwitchOn;
        log_.DBG1("MOTOR", "Controller %d state: Not ready to switch on", node_id_);
        return;
      }
    }
    if ((status << 4) == 16) {
      state_ = kReadyToSwitchOn;
      log_.DBG1("MOTOR", "Controller %d state: Ready to switch on", node_id_);
      return;
    }
    if ((status << 4) == 48) {
      state_ = kSwitchedOn;
      log_.DBG1("MOTOR", "Controller %d state: Switched on", node_id_);
      return;
    }
    if ((status << 4) == 112) {
      if (status & (1 << 5)) {
        state_ = kOperationEnabled;
        log_.DBG1("MOTOR", "Controller %d state: Operation enabled", node_id_);
        return;
      } else {
        state_ = kQuickStopActive;
        log_.DBG1("MOTOR", "Controller %d state: Quick stop active", node_id_);
        return;
      }
    }
    if ((status << 4) == 240) {
      state_ = kFaultReactionActive;
      log_.DBG1("MOTOR", "Controller %d state: Fault reaction active", node_id_);
      return;
    }
    if ((status << 4) == 64) {
      state_ = kFault;
      log_.DBG1("MOTOR", "Controller %d state: Fault", node_id_);
      return;
    }
    log_.ERR("MOTOR", "Controller %d: Status word not recognised", node_id_);
  }
}

void Controller::processNMTMessage(utils::io::can::Frame& message)
{
  int8_t nmt_state = message.data[0];
  switch (nmt_state) {
    case 0x00:
      log_.INFO("MOTOR", "Controller %d NMT State: Bootup", node_id_);
      break;
    case 0x04:
      log_.INFO("MOTOR", "Controller %d NMT State: Stopped", node_id_);
      break;
    case 0x05:
      log_.INFO("MOTOR", "Controller %d NMT State: Operational", node_id_);
      break;
    case 0x7F:
      log_.INFO("MOTOR", "Controller %d NMT State: Pre-Operational", node_id_);
      break;
    default:
      log_.ERR("MOTOR", "Controller %d NMT State: Not Recognised", node_id_);
  }
}

void Controller::quickStop()
{
  // Send quickStop command
  SDOMessage.data[0]   = kWRITE_2_BYTES;
  SDOMessage.data[1]   = 0x40;
  SDOMessage.data[2]   = 0x60;
  SDOMessage.data[3]   = 0x00;
  SDOMessage.data[4]   = 0x02;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller %d: Sending quickStop command", node_id_);
}

bool Controller::getFailure()
{
  return critical_failure_;
}

uint8_t Controller::getId()
{
  return node_id_;
}

ControllerState Controller::getControllerState()
{
  return state_;
}

}}  // namespace hyped::motor_control

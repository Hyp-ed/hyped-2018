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

#include "motor_control/controller.hpp"
#include <cstdint>

#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "data/data.hpp"
#include "utils/io/can.hpp"

namespace hyped {
namespace motor_control {

using utils::io::can::Frame;
using utils::concurrent::Thread;

/* All CAN messages in this class follow the CANOpen protocol. Types of messages
 * used here are Service Data Object Messages, Network Management Messages and Emergency
 * Messages. All messages are little edian.
 */

/* Service Data Object (SDO) messages provide access to the object dictionary of the controller
 * for reading/writing to the memory register.
 * SDO CAN Frames are composed as follows:
 * CAN ID:    COB-ID = Function + Node ID,
 * Len:       Size of object dictionary entry
 * Byte 0:    Command
 * Bytes 1-2: Object Dictionary Index
 * Byte  3:   Object Dictionary Sub-Index
 * Bytes 4-7: Data
 */

/* The Network Management (NMT) Protocol is used to start/stop and reset the Controller nodes in the system.
 * NMT CAN Frames composed as follows:
 * CAN ID: 0x000
 * Len:    2
 * Byte 1: NMT Function
 * Byte 2: Target Node
 */

// Types of CANopen messages, these are used for CAN ID's
constexpr uint32_t kEmgyTransmit          = 0x80;
constexpr uint32_t kSdoReceive            = 0x600;
constexpr uint32_t kSdoTransmit           = 0x580;
constexpr uint32_t kNmtReceive            = 0x000;
constexpr uint32_t kNmtTransmit           = 0x700;

// Function codes for sending messages
constexpr uint8_t  kReadObject            = 0x40;
constexpr uint8_t  kWriteOneByte          = 0x2F;
constexpr uint8_t  kWriteTwoBytes         = 0x2B;
// constexpr uint8_t  kWriteThreeBytes       = 0x27;  // TODO(anyone) add back in if needed
constexpr uint8_t  kWriteFourBytes        = 0x23;

// Network management commands
constexpr uint8_t  kNmtOperational        = 0x01;
// constexpr uint8_t  kNmtStop               = 0x02;  // TODO(anyone) add back in if needed
// constexpr uint8_t  kNmtPreOperational     = 0x80;  // TODO(anyone) add back in if needed
// constexpr uint8_t  kNmtResetNode          = 0x81;  // TODO(anyone) add back in if needed
// constexpr uint8_t  kNmtResetComms         = 0x82;  // TODO(anyone) add back in if needed

Controller::Controller(Logger& log, uint8_t id)
  : log_(log),
    can_(Can::getInstance()),
    data_(data::Data::getInstance()),
    motor_data_(data_.getMotorData()),
    state_(kNotReadyToSwitchOn),
    node_id_(id),
    critical_failure_(false),
    actual_velocity_(0),
    actual_torque_(0),
    sdo_frame_recieved_(false),
    motor_temperature_(0),
    controller_temperature_(0)
{
  sdo_message_.id       = kSdoReceive + node_id_;
  sdo_message_.extended = false;
  sdo_message_.len      = 8;
  nmt_message_.id       = kNmtReceive;
  nmt_message_.extended = false;
  nmt_message_.len      = 2;
  can_.start();
}

bool Controller::hasId(uint32_t id, bool extended)
{
  if (kEmgyTransmit + node_id_ == id) return true;
  if (kSdoTransmit  + node_id_ == id) return true;
  if (kNmtTransmit  + node_id_ == id) return true;
  return false;
}

void Controller::registerController()
{
  can_.registerProcessor(this);
}

void Controller::configure()
{
  log_.INFO("MOTOR", "Controller %d: Configuring...", node_id_);

  // Set motor pole pairs to 10
  sdo_message_.data[0]   = kWriteOneByte;
  sdo_message_.data[1]   = 0x33;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x0A;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring motor poles", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set feedback type to SSI
  sdo_message_.data[0]   = kWriteOneByte;
  sdo_message_.data[1]   = 0x40;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x01;
  sdo_message_.data[4]   = 0x02;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring feedback type", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set over voltage limit to 125
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0x54;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x7D;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring over voltage limit", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set under voltage limit to 25
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0x55;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x01;
  sdo_message_.data[4]   = 0x19;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring under voltage limit", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set under voltage minimum to 20
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0x55;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x03;
  sdo_message_.data[4]   = 0x14;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring under voltage minimum", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set motor temperature sensor
  sdo_message_.data[0]   = kWriteOneByte;
  sdo_message_.data[1]   = 0x57;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x01;
  sdo_message_.data[4]   = 0x03;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring motor temperature sensor", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set motor rated current to 800,000 mA
  sdo_message_.data[0]   = kWriteFourBytes;
  sdo_message_.data[1]   = 0x75;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x00;
  sdo_message_.data[5]   = 0x35;
  sdo_message_.data[6]   = 0x0C;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring motor rated current", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set motor rated torque to 180N
  sdo_message_.data[0]   = kWriteFourBytes;
  sdo_message_.data[1]   = 0x76;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x20;
  sdo_message_.data[5]   = 0xBF;
  sdo_message_.data[6]   = 0x02;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring motor rated torque", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set current control torque regulator P gain to 1200
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0xF6;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x01;
  sdo_message_.data[4]   = 0xB0;
  sdo_message_.data[5]   = 0x04;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring current control torque P gain", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set current control torque regulator I gain to 600
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0xF6;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x02;
  sdo_message_.data[4]   = 0x58;
  sdo_message_.data[5]   = 0x02;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring current control torque I gain", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set current control flux regulator P gain to 1200
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0xF6;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x03;
  sdo_message_.data[4]   = 0xB0;
  sdo_message_.data[5]   = 0x04;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring current control flux P gain", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set current control flux regulator I gain to 600
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0xF6;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x04;
  sdo_message_.data[4]   = 0x58;
  sdo_message_.data[5]   = 0x02;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring current control torque I gain", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set current control regulator ramp to 32000
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0xF6;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x05;
  sdo_message_.data[4]   = 0x00;
  sdo_message_.data[5]   = 0x7D;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring current control ramp", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set maximum controller current to 700,000 mA
  sdo_message_.data[0]   = kWriteFourBytes;
  sdo_message_.data[1]   = 0x50;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x60;
  sdo_message_.data[5]   = 0xAE;
  sdo_message_.data[6]   = 0x0A;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring maximum controller current", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set secondary controller current protection to 800,000 mA
  sdo_message_.data[0]   = kWriteFourBytes;
  sdo_message_.data[1]   = 0x51;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x00;
  sdo_message_.data[5]   = 0x35;
  sdo_message_.data[6]   = 0x0C;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring secondary current protection", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set maximum velocity to 7000 RPM
  sdo_message_.data[0]   = kWriteFourBytes;
  sdo_message_.data[1]   = 0x52;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x01;
  sdo_message_.data[4]   = 0x58;
  sdo_message_.data[5]   = 0x1B;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Configuring maximum RPM", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  log_.INFO("MOTOR", "Controller %d: Configured", node_id_);
}

void Controller::enterOperational()
{
  // Send NMT Operational message to transition from state 0 (Not ready to switch on)
  // to state 1 (Switch on disabled)
  nmt_message_.data[0]   = kNmtOperational;
  nmt_message_.data[1]   = node_id_;

  log_.INFO("MOTOR", "Controller %d: Sending NMT Operational command", node_id_);
  can_.send(nmt_message_);
  // Leave sufficient time for controller to enter NMT Operational
  Thread::sleep(100);

  // Enable velocity mode
  sdo_message_.data[0]   = kWriteOneByte;
  sdo_message_.data[1]   = 0x60;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x09;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Enabling velocity mode", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Set target velocity to 0;
  sendTargetVelocity(0);

  // Apply brake
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0x40;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x80;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Applying brake", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Send shutdown message to transition from state 1 (Switch on disabled)
  // to state 2 (Ready to switch on)
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0x40;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x06;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Shutdown command sent", node_id_);
  requestStateTransition(sdo_message_, kReadyToSwitchOn);

  // Send enter operational message to transition from state 2 (Ready to switch on)
  // to state 4 (Operation enabled)
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0x40;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x0F;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Enabling drive function", node_id_);
  requestStateTransition(sdo_message_, kOperationEnabled);
}

void Controller::enterPreOperational()
{
  checkState();
  if (state_ != kReadyToSwitchOn) {
    // Send shutdown command
    sdo_message_.data[0]   = kWriteTwoBytes;
    sdo_message_.data[1]   = 0x40;
    sdo_message_.data[2]   = 0x60;
    sdo_message_.data[3]   = 0x00;
    sdo_message_.data[4]   = 0x06;
    sdo_message_.data[5]   = 0x00;
    sdo_message_.data[6]   = 0x00;
    sdo_message_.data[7]   = 0x00;

    log_.DBG1("MOTOR", "Controller %d: Shutting down motor", node_id_);
    sendSdoMessage(sdo_message_);
    if (critical_failure_) {
      return;
    }
  }
}

void Controller::checkState()
{
  // Check Statusword in object dictionary
  sdo_message_.data[0]   = kReadObject;
  sdo_message_.data[1]   = 0x41;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x00;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Checking status", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }
}

void Controller::sendTargetVelocity(int32_t target_velocity)
{
  // Send 32 bit integer in Little Edian bytes
  sdo_message_.data[0]   = kWriteFourBytes;
  sdo_message_.data[1]   = 0xFF;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = target_velocity & 0xFF;
  sdo_message_.data[5]   = (target_velocity >> 8) & 0xFF;
  sdo_message_.data[6]   = (target_velocity >> 16) & 0xFF;
  sdo_message_.data[7]   = (target_velocity >> 24) & 0xFF;

  log_.DBG2("MOTOR", "Controller %d: Updating target velocity to %d", node_id_, target_velocity);
  can_.send(sdo_message_);
}

void Controller::sendTargetTorque(int16_t target_torque)
{
  // Send 32 bit integer in Little Edian bytes
  sdo_message_.data[0]   = kWriteFourBytes;
  sdo_message_.data[1]   = 0x71;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = target_torque & 0xFF;
  sdo_message_.data[5]   = (target_torque >> 8) & 0xFF;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG2("MOTOR", "Controller %d: Updating target torque tp %d", node_id_, target_torque);
  sendSdoMessage(sdo_message_);
}

void Controller::updateActualVelocity()
{
  // Check actual velocity in object dictionary
  sdo_message_.data[0]   = kReadObject;
  sdo_message_.data[1]   = 0x6C;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x00;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG2("MOTOR", "Controller %d: Reading actual velocity", node_id_);
  sendSdoMessage(sdo_message_);
}

void Controller::updateActualTorque()
{
  // Check actual velocity in object dictionary
  sdo_message_.data[0]   = kReadObject;
  sdo_message_.data[1]   = 0x77;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x00;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG2("MOTOR", "Controller %d: Reading actual torque", node_id_);
  sendSdoMessage(sdo_message_);
}

int32_t Controller::getVelocity()
{
  return actual_velocity_;
}

int16_t Controller::getTorque()
{
  return actual_torque_;
}

void Controller::quickStop()
{
  // Send quickStop command
  sdo_message_.data[0]   = kWriteTwoBytes;
  sdo_message_.data[1]   = 0x40;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x0B;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG1("MOTOR", "Controller %d: Sending quickStop command", node_id_);
  sendSdoMessage(sdo_message_);
}

void Controller::healthCheck()
{
  // Check warning status
  sdo_message_.data[0]   = kReadObject;
  sdo_message_.data[1]   = 0x27;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x00;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.INFO("MOTOR", "Controller %d: Checking for warnings", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }

  // Check error status
  sdo_message_.data[0]   = kReadObject;
  sdo_message_.data[1]   = 0x3F;
  sdo_message_.data[2]   = 0x60;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x00;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.INFO("MOTOR", "Controller %d: Checking for errors", node_id_);
  sendSdoMessage(sdo_message_);
  if (critical_failure_) {
    return;
  }
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

void Controller::processNewData(utils::io::can::Frame& message)
{
  uint32_t id = message.id;
  if (id == kEmgyTransmit + node_id_) {
    processEmergencyMessage(message);
  } else if (id == kSdoTransmit + node_id_) {
    processSdoMessage(message);
  } else if (id == kNmtTransmit + node_id_) {
    processNmtMessage(message);
  } else {
    log_.ERR("MOTOR", "Controller %d: CAN message not recognised", node_id_);
  }
}

void Controller::updateMotorTemp()
{
  // Check motor temp in object dictionary
  sdo_message_.data[0]   = kReadObject;
  sdo_message_.data[1]   = 0x25;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x00;
  sdo_message_.data[4]   = 0x00;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG2("MOTOR", "Controller %d: Reading motor temperature", node_id_);
  sendSdoMessage(sdo_message_);
}

void Controller::updateControllerTemp()
{
  // Check controller temp in object dictionary
  sdo_message_.data[0]   = kReadObject;
  sdo_message_.data[1]   = 0x26;
  sdo_message_.data[2]   = 0x20;
  sdo_message_.data[3]   = 0x01;
  sdo_message_.data[4]   = 0x00;
  sdo_message_.data[5]   = 0x00;
  sdo_message_.data[6]   = 0x00;
  sdo_message_.data[7]   = 0x00;

  log_.DBG2("MOTOR", "Controller %d: Reading controller temperature", node_id_);
  sendSdoMessage(sdo_message_);
}

uint8_t Controller::getMotorTemp()
{
  return motor_temperature_;
}

uint8_t Controller::getControllerTemp()
{
  return controller_temperature_;
}

void Controller::sendSdoMessage(utils::io::can::Frame& message)
{
  sdo_frame_recieved_ = false;
  int8_t send_counter = 0;
  for (send_counter = 0; send_counter < 3; send_counter++) {
    can_.send(message);
    Thread::yield();
    if (sdo_frame_recieved_) {
      break;
    } else {
      log_.DBG1("MOTOR", "Controller %d: No response. Sending SDO frame again", node_id_);
    }
  }
  // No SDO frame recieved - controller must be offline/communication error
  if (!sdo_frame_recieved_) {
    log_.ERR("MOTOR", "Controller %d: No response from controller", node_id_);
    throwCriticalFailure();
  }
}

void Controller::throwCriticalFailure()
{
  critical_failure_ = true;
  motor_data_ = data_.getMotorData();
  motor_data_.module_status = data::ModuleStatus::kCriticalFailure;
  data_.setMotorData(motor_data_);
}

void Controller::requestStateTransition(utils::io::can::Frame& message, ControllerState state)
{
  uint8_t state_count;
  // Wait for maximum of three seconds, checking state each second.
  // If state doesn't change then throw critical failure
  for (state_count = 0; state_count < 3; state_count++) {
    can_.send(message);
    Thread::sleep(1000);
    checkState();
    if (state_ == state) {
      return;
    }
  }
  if (state_ != state) {
    throwCriticalFailure();
    log_.ERR("MOTOR", "Controller %d, Could not transition to state %d", node_id_, state);
    return;
  }
}

void Controller::processEmergencyMessage(utils::io::can::Frame& message)
{
  log_.ERR("MOTOR", "Controller %d: CAN Emergency", node_id_);
  throwCriticalFailure();
  uint8_t index_1   = message.data[0];
  uint8_t index_2   = message.data[1];

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

  if (index_2 == 0xFF || index_1 == 0xFF) {
    log_.ERR("MOTOR", "Controller %d: Manufacturer error", node_id_);
    // Manufacturer specific error calling processErrorMessage
    log_.ERR("MOTOR", "Calling process error function");
    uint16_t error_message = (index_2 << 8) | index_1;
    processErrorMessage(error_message);
  }
  log_.DBG1("MOTOR", "index 1: %d, index 2: %d", index_1, index_2);
}

void Controller::processErrorMessage(uint16_t error_message)
{
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

void Controller::processSdoMessage(utils::io::can::Frame& message)
{
  sdo_frame_recieved_ = true;
  uint8_t index_1   = message.data[1];
  uint8_t index_2   = message.data[2];
  uint8_t sub_index = message.data[3];

  // Process actual velocity
  if (index_1 == 0x6C && index_2 == 0x60) {
    actual_velocity_ =  ((int32_t) message.data[7]) << 24
                      | ((int32_t) message.data[6]) << 16
                      | ((int32_t) message.data[5]) << 8
                      | message.data[4];
    return;
  }

  // Process actual torque
  if (index_1 == 0x77 && index_2 == 0x60) {
    actual_torque_   = ((int16_t) message.data[5]) << 8 | message.data[4];
    return;
  }

  // Process motor temperature
  if (index_1 == 0x25 && index_2 == 0x20) {
    motor_temperature_ = message.data[4];
  }

  // Process controller temperature
  if (index_1 == 0x26 && index_2 == 0x20 && sub_index == 0x01) {
    controller_temperature_ = message.data[4];
  }

  // Process warning message
  if (index_1 == 0x27 && index_2 == 0x20 && sub_index == 0x00) {
    if (message.data[4] != 0 && message.data[5] != 0) {
      throwCriticalFailure();
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
      uint16_t error_message = (message.data[5] << 8) | message.data[4];
      throwCriticalFailure();
      processErrorMessage(error_message);
    }
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
    switch (status) {
    case 0x00:
      state_ = kNotReadyToSwitchOn;
      log_.DBG1("MOTOR", "Controller %d state: Not ready to switch on", node_id_);
      break;
    case 0x40:
      state_ = kSwitchOnDisabled;
      log_.DBG1("MOTOR", "Controller %d state: Switch on disabled", node_id_);
      break;
    case 0x21:
      state_ = kReadyToSwitchOn;
      log_.DBG1("MOTOR", "Controller %d state: Ready to switch on", node_id_);
      break;
    case 0x23:
      state_ = kSwitchedOn;
      log_.DBG1("MOTOR", "Controller %d state: Switched on", node_id_);
      break;
    case 0x27:
      state_ = kOperationEnabled;
      log_.DBG1("MOTOR", "Controller %d state: Operation enabled", node_id_);
      break;
    case 0x07:
      state_ = kQuickStopActive;
      log_.DBG1("MOTOR", "Controller %d state: Quick stop active", node_id_);
      break;
    case 0x0F:
      state_ = kFaultReactionActive;
      log_.DBG1("MOTOR", "Controller %d state: Fault reaction active", node_id_);
      break;
    case 0x08:
      state_ = kFault;
      log_.DBG1("MOTOR", "Controller %d state: Fault", node_id_);
      break;
    default:
      log_.DBG1("MOTOR", "Controller %d state: State not recognised", node_id_);
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
  if (index_1 == 0x40 && index_2 == 0x20 && sub_index == 0x02) {
    log_.DBG1("MOTOR", "Controller %d: Motor phase offset configured", node_id_);
    return;
  }
  if (index_1 == 0x40 && index_2 == 0x20 && sub_index == 0x08) {
    log_.DBG1("MOTOR", "Controller %d: Motor phase offset compensation configured", node_id_);
    return;
  }
  if (index_1 == 0x54 && index_2 == 0x20 && sub_index == 0x00) {
    log_.DBG1("MOTOR", "Controller %d: Over voltage limit configured", node_id_);
    return;
  }
  if (index_1 == 0x55 && index_2 == 0x20 && sub_index == 0x03) {
    log_.DBG1("MOTOR", "Controller %d: Under voltage minimum configured", node_id_);
    return;
  }
  if (index_1 == 0x55 && index_2 == 0x20 && sub_index == 0x01) {
    log_.DBG1("MOTOR", "Controller %d: Under voltage limit configured", node_id_);
    return;
  }
  if (index_1 == 0x57 && index_2 == 0x20 && sub_index == 0x01) {
    log_.DBG1("MOTOR", "Controller %d: Temperature sensor configured", node_id_);
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
  if (index_1 == 0xF6 && index_2 == 0x60 && sub_index == 0x01) {
    log_.DBG1("MOTOR", "Controller %d: Current control torque P gain configured", node_id_);
    return;
  }
  if (index_1 == 0xF6 && index_2 == 0x60 && sub_index == 0x02) {
    log_.DBG1("MOTOR", "Controller %d: Current control torque I gain configured", node_id_);
    return;
  }
  if (index_1 == 0xF6 && index_2 == 0x60 && sub_index == 0x03) {
    log_.DBG1("MOTOR", "Controller %d: Current control flux P gain configured", node_id_);
    return;
  }
  if (index_1 == 0xF6 && index_2 == 0x60 && sub_index == 0x04) {
    log_.DBG1("MOTOR", "Controller %d: Current control flux I gain configured", node_id_);
    return;
  }
  if (index_1 == 0xF6 && index_2 == 0x60 && sub_index == 0x05) {
    log_.DBG1("MOTOR", "Controller %d: Current control ramp configured", node_id_);
    return;
  }
  if (index_1 == 0x50 && index_2 == 0x20 && sub_index == 0x00) {
    log_.DBG1("MOTOR", "Controller %d: Maximum current limit configured", node_id_);
    return;
  }
  if (index_1 == 0x51 && index_2 == 0x20 && sub_index == 0x00) {
    log_.DBG1("MOTOR", "Controller %d: Secondary current protection configured", node_id_);
    return;
  }
  if (index_1 == 0x52 && index_2 == 0x20 && sub_index == 0x01) {
    log_.DBG1("MOTOR", "Controller %d: Maximum RPM configured", node_id_);
    return;
  }

  // Controlword updates
  if (index_1 == 0x40 && index_2 == 0x60 && sub_index == 0x00) {
    log_.DBG1("MOTOR", "Controller %d: Control Word updated", node_id_);
    return;
  }
}

void Controller::processNmtMessage(utils::io::can::Frame& message)
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

}}  // namespace hyped::motor_control

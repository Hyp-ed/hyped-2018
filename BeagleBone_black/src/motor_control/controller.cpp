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

// Status masks, used to mask the return result of a status check
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
    error_status_checked_(true),
    error_(false),
    actual_velocity_(0),
    actual_torque_(0),
    configure_count_(0),
    config_error_count_(0),
    configured_(false),
    state(kNotReadyToSwitchOn)
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
  log_.INFO("MOTOR", "Controller : Configuring...");

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
  log_.DBG1("MOTOR", "Controller : Configuring motor poles");

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
  log_.DBG1("MOTOR", "Controller : Configuring feedback type");

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
  log_.DBG1("MOTOR", "Controller : Configuring motor phase offset");

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
  log_.DBG1("MOTOR", "Controller : Configuring over voltage limit");

  // Set under voltage minimum to 30
  SDOMessage.data[0]   = kWRITE_2_BYTES;
  SDOMessage.data[1]   = 0x55;
  SDOMessage.data[2]   = 0x20;
  SDOMessage.data[3]   = 0x03;
  SDOMessage.data[4]   = 0x1E;
  SDOMessage.data[5]   = 0x00;
  SDOMessage.data[6]   = 0x00;
  SDOMessage.data[7]   = 0x00;

  can_.send(SDOMessage);
  log_.DBG1("MOTOR", "Controller : Configuring under voltage limit");

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
  log_.DBG1("MOTOR", "Controller : Configuring motor rated current");

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
  log_.DBG1("MOTOR", "Controller : Configuring motor rated torque");

  // Reset node after configuration
  NMTMessage.data[0]   = kNMT_RESET_NODE;
  NMTMessage.data[1]   = node_id_;

  can_.send(NMTMessage);
  log_.DBG1("MOTOR", "Controller : Reset node");

  // Wait for 10 x 200ms for all 7 configuration confirmation messages to return.
  // If we wait for 2 full seconds and there are less than 7 configuration confirmations,
  // then we throw a configuarion error
  while (configure_count_ != 7 && config_error_count_ != 10) {
    Thread::sleep(200);
    config_error_count_++;
  }

  if (config_error_count_ == 10) {
    log_.ERR("MOTOR", "Configuration error");  // TODO(Anyone) handle configution error
  }

  configured_ = true;
}

void Controller::enterOperational()
{
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
  log_.INFO("MOTOR", "Controller : Checking for warnings");

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
  log_.INFO("MOTOR", "Controller : Checking for errors");

  // Wait until warning and error status is checked
  while (!error_status_checked_);

  if (!error_) {
    // Enter NMT Operational
    SDOMessage.data[0]   = kNMT_OPERATIONAL;
    SDOMessage.data[1]   = node_id_;

    can_.send(NMTMessage);
    log_.INFO("MOTOR", "Controller : NMT Operational command sent");

    // Send shutdown message to tranition state (Disable drive function)
    SDOMessage.data[0]   = kWRITE_2_BYTES;
    SDOMessage.data[1]   = 0x40;
    SDOMessage.data[2]   = 0x60;
    SDOMessage.data[3]   = 0x00;
    SDOMessage.data[4]   = 0x06;
    SDOMessage.data[5]   = 0x00;
    SDOMessage.data[6]   = 0x00;
    SDOMessage.data[7]   = 0x00;

    can_.send(SDOMessage);
    log_.DBG1("MOTOR", "Controller : Disabling drive function");

    // Send switch on message (Disable breaks)
    SDOMessage.data[0]   = kWRITE_2_BYTES;
    SDOMessage.data[1]   = 0x40;
    SDOMessage.data[2]   = 0x60;
    SDOMessage.data[3]   = 0x00;
    SDOMessage.data[4]   = 0x07;
    SDOMessage.data[5]   = 0x00;
    SDOMessage.data[6]   = 0x00;
    SDOMessage.data[7]   = 0x00;

    can_.send(SDOMessage);
    log_.DBG1("MOTOR", "Controller : Disabling breaks");

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
    log_.DBG1("MOTOR", "Controller : Enabling drive function");

    // Enable velocity mode
    SDOMessage.data[0]   = kWRITE_2_BYTES;
    SDOMessage.data[1]   = 0x40;
    SDOMessage.data[2]   = 0x60;
    SDOMessage.data[3]   = 0x00;
    SDOMessage.data[4]   = 0x03;
    SDOMessage.data[5]   = 0x00;
    SDOMessage.data[6]   = 0x00;
    SDOMessage.data[7]   = 0x00;

    can_.send(SDOMessage);
    log_.DBG1("MOTOR", "Controller : Enabling velocity mode");

    // Check if controller is in Operational state
    this->checkStatus();
  } else {
    log_.ERR("MOTOR", "Controller : ERROR");  // TODO(Anyone): Handle error
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
  log_.DBG1("MOTOR", "Controller : Checking status");
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
  log_.DBG2("MOTOR", "Updating target velocity");
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
  log_.DBG2("MOTOR", "Updating target torque");
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
  log_.DBG2("MOTOR", "Controller : Reading actual velocity");
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
  log_.DBG2("MOTOR", "Controller : Reading actual torque");
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
    log_.ERR("MOTOR", "CAN message not recognised");
  }
}

void Controller::processEmergencyMessage(utils::io::can::Frame& message)
{/*EMPTY*/}

void Controller::processSDOMessage(utils::io::can::Frame& message)
{
  // Process configuration messages
  if (message.data[1] == 0x33 && message.data[2] == 0x20 && message.data[3] == 0x00) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller : Motor poles configured");
    return;
  }
  if (message.data[1] == 0x40 && message.data[2] == 0x20 && message.data[3] == 0x01) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller : Feedback type configured");
    return;
  }
  if (message.data[1] == 0x40 && message.data[2] == 0x20 && message.data[3] == 0x08) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller : Motor phase offset configured");
    return;
  }
  if (message.data[1] == 0x54 && message.data[2] == 0x20 && message.data[3] == 0x00) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller : Over voltage limit configured");
    return;
  }
  if (message.data[1] == 0x55 && message.data[2] == 0x20 && message.data[3] == 0x03) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller : Under voltage limit configured");
    return;
  }
  if (message.data[1] == 0x75 && message.data[2] == 0x60 && message.data[3] == 0x00) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller : Motor rated current configured");
    return;
  }
  if (message.data[1] == 0x76 && message.data[2] == 0x60 && message.data[3] == 0x00) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller : Motor rated torque configured");
    return;
  }

  // Process warning and error messages
  if (message.data[1] == 0x27 && message.data[2] == 0x20 && message.data[3] == 0x00) {
    if (message.data[4] + message.data[5] != 0) {
      error_ = true;
    }
    return;
  }
  if (message.data[1] == 0x3F && message.data[2] == 0x60 && message.data[3] == 0x00) {
    if (message.data[4] + message.data[5] != 0) {
      error_ = true;
    }
    error_status_checked_ = true;
    return;
  }

  // Controlword updates
  if (message.data[1] == 0x40 && message.data[2] == 0x60 && message.data[3] == 0x00) {
    log_.DBG1("MOTOR", "Controller : Control Word updated");
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
  if (message.data[1] == 0x41 && message.data[2] == 0x60 && message.data[3] == 0x00) {
    uint8_t status = message.data[4];
    if ((status << 4) == 0) {
      if (status & (1 << 6)) {
        state = kSwitchOnDisabled;
        log_.DBG1("MOTOR", "Controller state: Switch on disabled");
        return;
      } else {
        state = kNotReadyToSwitchOn;
        log_.DBG1("MOTOR", "Controller state: Not ready to switch on");
        return;
      }
    }
    if ((status << 4) == 16) {
      state = kReadyToSwitchOn;
      log_.DBG1("MOTOR", "Controller state: Ready to switch on");
      return;
    }
    if ((status << 4) == 48) {
      state = kSwitchedOn;
      log_.DBG1("MOTOR", "Controller state: Switched on");
      return;
    }
    if ((status << 4) == 112) {
      if (status & (1 << 5)) {
        state = kOperationEnabled;
        log_.DBG1("MOTOR", "Controller state: Operation enabled");
        return;
      } else {
        state = kQuickStopActive;
        log_.DBG1("MOTOR", "Controller state: Quick stop active");
        return;
      }
    }
    if ((status << 4) == 240) {
      state = kFaultReactionActive;
      log_.DBG1("MOTOR", "Controller state: Fault reaction active");
      return;
    }
    if ((status << 4) == 64) {
      state = kFault;
      log_.DBG1("MOTOR", "Controller state: Fault");
      return;
    }
    log_.ERR("MOTOR", "Status word not recognised");
  }
}

void Controller::processNMTMessage(utils::io::can::Frame& message)
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

bool Controller::getConfiguartionStatus()
{
  return configured_;
}

}}  // namespace hyped::motor_control

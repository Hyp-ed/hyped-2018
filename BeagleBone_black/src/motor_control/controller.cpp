/*
 * Author: Sean Mullan
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
  log_.DBG1("MOTOR", "Controller %d: Configuring under voltage limit", node_id_);

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

  // Reset node after configuration
  NMTMessage.data[0]   = kNMT_RESET_NODE;
  NMTMessage.data[1]   = node_id_;

  can_.send(NMTMessage);
  log_.DBG1("MOTOR", "Controller %d: Reset node", node_id_);

  // Wait for 10 x 100ms for all 7 configuration confirmation messages to return.
  // If we wait for 1 full second and there are less than 7 configuration confirmations,
  // then we throw a configuarion error
  while (configure_count_ != 7 && config_error_count_ != 10) {
    Thread::sleep(100);
    config_error_count_++;
  }

  if (config_error_count_ == 10) {
    log_.ERR("MOTOR", "Controller %d: Configuration error", node_id_);
    // TODO(Anyone) handle configution error
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

  // Wait until warning and error status is checked
  while (!error_status_checked_);

  if (!error_) {
    // Enter NMT Operational
    SDOMessage.data[0]   = kNMT_OPERATIONAL;
    SDOMessage.data[1]   = node_id_;

    can_.send(NMTMessage);
    log_.INFO("MOTOR", "Controller %d: NMT Operational command sent", node_id_);

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
    log_.DBG1("MOTOR", "Controller %d: Disabling drive function", node_id_);

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
    log_.DBG1("MOTOR", "Controller %d: Disabling breaks", node_id_);

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
    log_.DBG1("MOTOR", "Controller %d: Enabling velocity mode", node_id_);

    // Check if controller is in Operational state
    this->checkStatus();
  } else {
    log_.ERR("MOTOR", "Controller %d: ERROR", node_id_);  // TODO(Anyone): Handle error
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
{/*EMPTY*/}

void Controller::processSDOMessage(utils::io::can::Frame& message)
{
  // Process actual velocity. TODO(Anyone) Cover negative velocity case
  if (message.data[1] == 0x6C && message.data[2] == 0x60) {
    actual_velocity_ = (message.data[7] << 24
                      | message.data[6] << 16
                      | message.data[5] << 8
                      | message.data[4]);
  }

  // Process actual torque. //TODO(Anyone) Cover negative torque case
  if (message.data[1] == 0x77 && message.data[2] == 0x60) {
    actual_torque_   = (message.data[5] << 8
                      | message.data[4]);
  }

  // Process configuration messages
  if (message.data[1] == 0x33 && message.data[2] == 0x20 && message.data[3] == 0x00) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller %d: Motor poles configured", node_id_);
    return;
  }
  if (message.data[1] == 0x40 && message.data[2] == 0x20 && message.data[3] == 0x01) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller %d: Feedback type configured", node_id_);
    return;
  }
  if (message.data[1] == 0x40 && message.data[2] == 0x20 && message.data[3] == 0x08) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller %d: Motor phase offset configured", node_id_);
    return;
  }
  if (message.data[1] == 0x54 && message.data[2] == 0x20 && message.data[3] == 0x00) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller %d: Over voltage limit configured", node_id_);
    return;
  }
  if (message.data[1] == 0x55 && message.data[2] == 0x20 && message.data[3] == 0x03) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller %d: Under voltage limit configured", node_id_);
    return;
  }
  if (message.data[1] == 0x75 && message.data[2] == 0x60 && message.data[3] == 0x00) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller %d: Motor rated current configured", node_id_);
    return;
  }
  if (message.data[1] == 0x76 && message.data[2] == 0x60 && message.data[3] == 0x00) {
    configure_count_++;
    log_.DBG1("MOTOR", "Controller %d: Motor rated torque configured", node_id_);
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
  if (message.data[1] == 0x41 && message.data[2] == 0x60 && message.data[3] == 0x00) {
    uint8_t status = message.data[4];
    if ((status << 4) == 0) {
      if (status & (1 << 6)) {
        state = kSwitchOnDisabled;
        log_.DBG1("MOTOR", "Controller %d state: Switch on disabled", node_id_);
        return;
      } else {
        state = kNotReadyToSwitchOn;
        log_.DBG1("MOTOR", "Controller %d state: Not ready to switch on", node_id_);
        return;
      }
    }
    if ((status << 4) == 16) {
      state = kReadyToSwitchOn;
      log_.DBG1("MOTOR", "Controller %d state: Ready to switch on", node_id_);
      return;
    }
    if ((status << 4) == 48) {
      state = kSwitchedOn;
      log_.DBG1("MOTOR", "Controller %d state: Switched on", node_id_);
      return;
    }
    if ((status << 4) == 112) {
      if (status & (1 << 5)) {
        state = kOperationEnabled;
        log_.DBG1("MOTOR", "Controller %d state: Operation enabled", node_id_);
        return;
      } else {
        state = kQuickStopActive;
        log_.DBG1("MOTOR", "Controller %d state: Quick stop active", node_id_);
        return;
      }
    }
    if ((status << 4) == 240) {
      state = kFaultReactionActive;
      log_.DBG1("MOTOR", "Controller %d state: Fault reaction active", node_id_);
      return;
    }
    if ((status << 4) == 64) {
      state = kFault;
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
  // TODO(Anyone) Add quickStop command
}

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

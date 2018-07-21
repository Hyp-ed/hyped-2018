/*
 * Author: Martin Kristien, Jack Horsburgh and Ragnor Comerford
 * Organisation: HYPED
 * Date: 13/03/18
 * Description:
 * Main manages sensor drivers, collects data from sensors and updates
 * shared Data::Sensors structure. Main is not responsible for initialisation
 * of supporting io drivers (i2c, spi, can). This should be done by the sensor
 * drivers themselves.
 * Currently supported sensors:
 * - BMS (low powered), ids: 0
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

#ifndef BEAGLEBONE_BLACK_SENSORS_MAIN_HPP_
#define BEAGLEBONE_BLACK_SENSORS_MAIN_HPP_

#include <cstdint>
// #include <memory>

#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include "sensors/gpio_counter.hpp"
#include "sensors/fake_gpio_counter.hpp"
#include "sensors/interface.hpp"
#include "sensors/manager_interface.hpp"
#include "utils/system.hpp"


namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;

namespace sensors {

class CANProxi;
class Keyence;

class Main: public Thread {
 public:
  Main(uint8_t id, Logger& log);
  void run() override;

 private:
  // check if all battery values are in expected range
  // returns true iff all battery values (LP and HP) are in expected ranges
  bool batteriesInRange();

  data::Data&     data_;
  utils::System& sys_;

  // master data structures
  data::Sensors   sensors_;
  data::Batteries batteries_;
  data::StripeCounter stripe_counter_;


  GpioInterface*                         keyence_l_;
  GpioInterface*                         keyence_r_;
  std::unique_ptr<ImuManagerInterface>   imu_manager_;
  #ifdef PROXI
  std::unique_ptr<ProxiManagerInterface> proxi_manager_front_;
  std::unique_ptr<ProxiManagerInterface> proxi_manager_back_;
  #endif
  std::unique_ptr<ManagerInterface>      battery_manager_;
  GpioInterface*                         optical_encoder_l_;
  GpioInterface*                         optical_encoder_r_;

  bool sensor_init_;
  bool battery_init_;
};

}}  // namespace hyped::sensors

#endif  // BEAGLEBONE_BLACK_SENSORS_MAIN_HPP_

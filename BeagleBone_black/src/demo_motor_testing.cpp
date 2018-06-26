/*
 * Author: Sean Mullan and Jack Horsburgh
 * Organisation: HYPED
 * Date: 28/03/18
 * Description: This demo runs a single motor. It logs RPM, motor temperature and controller temperature data.
 *              Both target velocity commands and data updates are executed every 50 milliseconds.
 *              The arguments are passed in as follows (Time units are in seconds):
 *              Max RPM, 
 *              Time to accelerate max RPM, 
 *              Time spent constant at max RPM, 
 *              Time to decelerate from max RPM to 0.
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

#include "motor_control/controller.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <chrono>

using hyped::utils::Logger;
using hyped::utils::System;
using hyped::utils::concurrent::Thread;
using hyped::motor_control::Controller;

using namespace std;
using namespace hyped;

Controller* controller;
ofstream    RPMvTime;
ofstream    Temperature;
int32_t     target_v;
int32_t     actual_v;
uint16_t    motor_temp;
uint16_t    controller_temp;
auto start = chrono::steady_clock::now();

void updateVelocityData()
{
  controller->updateActualVelocity();
  actual_v = controller->getVelocity();
  auto now = chrono::steady_clock::now();
  RPMvTime << chrono::duration_cast<chrono::milliseconds>(now-start).count() << "\t" << actual_v << "\n";
}

void updateTemperatureData()
{
  controller->updateMotorTemp();
  controller->updateControllerTemp();
  motor_temp =      (uint16_t) controller->getMotorTemp();
  controller_temp = (uint16_t) controller->getControllerTemp();
  auto now = chrono::steady_clock::now();
  Temperature << chrono::duration_cast<chrono::milliseconds>(now-start).count() << "\t" << motor_temp << "\t" << controller_temp << "\n";
}

int main(int argc, char* argv[])
{
  System::parseArgs(argc, argv);
  System& sys = System::getSystem();
  Logger log_motor(sys.verbose_motor, sys.debug_motor);

  Logger log(true, 1);
  controller = new Controller(log,1);

  RPMvTime.open ("RPMvTime.txt");
  RPMvTime << "Time\tRPM\n";

  Temperature.open ("Temperature.txt");
  Temperature << "Time\tMotor\tController\n";

  int32_t max_rpm   = atoi(argv[1]);  // Desired max rpm
  int16_t acc_time  = atoi(argv[2]);  // Time (seconds) to max rpm
  int16_t con_time  = atoi(argv[3]);  // Time (seconds) spent at max rpm
  int16_t dec_time  = atoi(argv[4]);  // Time (seconds) from max rpm to 0
  int32_t wait      = 50;             // Wait 50ms between each target velocity update
  
  target_v        = 0;
  actual_v        = 0;
  motor_temp      = 0;
  controller_temp = 0;

  int acc_iterations = acc_time*20;
  int con_iterations = con_time*20;
  int dec_iterations = dec_time*20;

  int32_t acc_updates = max_rpm/acc_iterations;
  int32_t dec_updates = max_rpm/dec_iterations;
  
  controller->registerController();
  controller->configure();
  controller->enterOperational();
  
  start = chrono::steady_clock::now();

  // Acceleration period
  for (int i = 0; i < acc_iterations; i++) {
    target_v += acc_updates;
    controller->sendTargetVelocity(target_v);
    updateVelocityData();
    updateTemperatureData();
    printf("Motor RPM: %d,    Motor Temperature: %d,    Controller Temperature: %d\n", actual_v, motor_temp, controller_temp);
    while(chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now()-start).count() < wait);
    wait += 50;
  }

  // Constant max rpm period
  for (int i = 0; i < con_iterations; i++) {
    updateVelocityData();
    updateTemperatureData();
    printf("Motor RPM: %d,    Motor Temperature: %d,    Controller Temperature: %d\n", actual_v, motor_temp, controller_temp);
    while(chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now()-start).count() < wait);
    wait += 50;
  }

  // Deceleration period
  for (int i = 0; i < dec_iterations; i++) {
    target_v -= dec_updates;
    controller->sendTargetVelocity(target_v);
    updateVelocityData();
    updateTemperatureData();
    printf("Motor RPM: %d,    Motor Temperature: %d,    Controller Temperature: %d\n", actual_v, motor_temp, controller_temp);
    while(chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now()-start).count() < wait);
    wait += 50;
  }
  
  controller->enterPreOperational();
  RPMvTime.close();
  Temperature.close();
}
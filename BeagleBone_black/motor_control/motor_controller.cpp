/*
 * Author: Sean Mullan and Jack Horsburgh
 * Organisation: HYPED
 * Date: 17/02/18
 * Description:
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

#include "motor.hpp"
#include "motor_controller.hpp"
#include <iostream>

MotorController::MotorController() 
{
  motor = new Motor();
  // Variables for testing
  rpm = 0;
  current_distance = 0;
  translational_velocity = 0;
}

/**
  *  @brief  { Establish CAN connections with motor controllers }
  */
void MotorController::setupMotors() 
{
  std::cout << "CAN connections established" << std::endl;
}

/**
  *  @brief  { Will accelerate motors until maximum acceleration distance is reached }
  */
void MotorController::accelerateMotors() 
{
  // Current distance will be continuosly read from shared data structure
  while(current_distance <= 500) {
    // Read translational velocity from shared data structure
    rpm = calculateAccelerationRPM(translational_velocity);
    motor->setSpeed(rpm);
    // Update test variables
    current_distance += 100;
    translational_velocity += 1000;
  }
}

/**
  *  @brief  { Will decelerate motors until total distance is reached }
  */
void MotorController::decelerateMotors() 
{
  while(current_distance <= 1000) {
    // Read translational velocity from shared data structure
    rpm = calculateDecelerationRPM(translational_velocity);
    motor->setSpeed(rpm);
    // Update test variables
    current_distance += 100;
    translational_velocity -= 1000;
  }
}

void MotorController::stopMotors() 
{
  motor->setSpeed(0);
  std::cout << "Motors stopped" << std::endl;
}

/**
  *  @brief  { This function will run through slip ratio algorithm to calculate
  *            the desired acceleration RPM }
  *
  *  @param[in]  translational_velocity  { Value read from shared data structure }
  *
  *  @return  { Acceleration RPM calculation of type int }
  */
int MotorController::calculateAccelerationRPM(double translational_velocity) 
{
  //dummy calculation to increase rpm
  return rpm += 1000;
}

/**
  *  @brief  { This function will run through slip ratio algorithm to calculate
  *            the desired deceleration RPM }
  *
  *  @param[in]  translational_velocity  { Value read from shared data structure }
  *
  *  @return  { Deceleration RPM calculation of type int }
  */
int MotorController::calculateDecelerationRPM(double translational_velocity) 
{
  //dummy function to decrease rpm
  return rpm -= 1000;
}

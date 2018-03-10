/*
 * Author: E. van Woerkom
 * Organisation: HYPED
 * Date: 10 March 2018
 * Description:
 * Header file for minimal I2C Project
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
#ifndef MINIMALI2C_MINIMAL_H_
#define MINIMALI2C_MINIMAL_H_

#include <asf.h>
#include <status_codes.h>

#define BUFFER_LENGTH 64 // Buffer length for read and write buffers
#define SLAVE_ADDRESS 0x29 // This is the VL6180X standard I2C address

/**
 * @brief      Configures I2C Bus
 */
void configure_i2c(void);
/**
 * @brief      Switches on VL6180X
 */
void turnon_routine(void);

/**
 * @brief      Blocking write
 * @return     Status of operation
 */
enum status_code writeDirect(struct i2c_master_module*, uint16_t, uint16_t, uint8_t*);

/**
 * @brief      Blocking read, enduser should never use this
 * @return     Status of operation
 */
enum status_code readDirect(struct i2c_master_module*, uint16_t, uint16_t, uint8_t*);

/**
 * @brief      Blocking write and read to get data from register
 * @return     Status of operation
 */
enum status_code getDirect(struct i2c_master_module*, uint16_t, uint16_t, uint16_t, uint8_t*, uint8_t*);

#endif // MINIMALI2C_MINIMAL_H_
/*
 * Author: E. van Woerkom
 * Organisation: HYPED
 * Date: 4 March 2018
 * Description:
 * Minimal I2C communication setup with three blocking functions
 * for interaction with VL6180X. This set up will continuously read the first
 * 64 bytes of memory of the VL6180X, which can be verified using the debugger.
 * Inspiration and the I2C configure function were taken from the ATMEL I2C example project
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

/**
 * \file
 *
 * \brief SAM SERCOM I2C Master Quick Start Guide with Callbacks
 *
 * Copyright (C) 2012-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/* USAGE INSTRUCTIONS:
 * To use this to setup communications with the VL6180X,
 * Wire-up the GPIO pin to PA08, SDA to PB30, SCL to PB31, VIN to VCC, GND to GND.
 */
#include "minimal.h"
#include <asf.h>
#include <delay.h>
#include <status_codes.h>

static uint8_t wr_buffer[BUFFER_LENGTH];
static uint8_t rd_buffer[BUFFER_LENGTH];

struct i2c_master_module i2c_master_instance; // This struct will 'Be' the configured I2C device

void configure_i2c(void)
{
  /* Initialize config structure and software module */
  struct i2c_master_config config_i2c_master;
  i2c_master_get_config_defaults(&config_i2c_master);
  // Set below specifics
  config_i2c_master.buffer_timeout = 65535;
  config_i2c_master.pinmux_pad0    = PINMUX_PB30D_SERCOM5_PAD0; // SDA
  config_i2c_master.pinmux_pad1    = PINMUX_PB31D_SERCOM5_PAD1; // SCL
  config_i2c_master.baud_rate      = I2C_MASTER_BAUD_RATE_400KHZ; // SHould be 400KHz for vl6180x

  /* Initialize and enable device with config */
  while(i2c_master_init(&i2c_master_instance, SERCOM5, &config_i2c_master)!= STATUS_OK);

  // enable_module
  i2c_master_enable(&i2c_master_instance);
}

// In order to boot the VL6180X pin PA08 connected to the GPIO must be switched off then on.
void turnon_routine(){ // Switches on VL6180X
  struct port_config config_port_pin; // make pin structure
  port_get_config_defaults(&config_port_pin);
  config_port_pin.direction  = PORT_PIN_DIR_OUTPUT;
  port_pin_set_config(PIN_PA08, &config_port_pin);
  
  // this sequence boots:
  port_pin_set_output_level(PIN_PA08, 0);
  delay_ms(1);
  port_pin_set_output_level(PIN_PA08, 1);
  // allow time for boot, VL6180X has a minimal boot time
  delay_ms(1);
}

// This function does a single blocking write operation to the I2C slave.
enum status_code writeDirect(struct i2c_master_module* i2c_device,
							 uint16_t slaveaddr, uint16_t datalen, uint8_t* data){
  struct i2c_master_packet writepacket;
  writepacket.address     = slaveaddr;
  writepacket.data_length = datalen;
  writepacket.data        = data;
  writepacket.ten_bit_address = 0; // These three settings must be set to zero or they will
  writepacket.hs_master_code = 0; // by their default values wreck the communications.
  writepacket.high_speed = 0;
  return i2c_master_write_packet_wait(i2c_device, &writepacket);
}

// This function does a single blocking read operation to the I2C slave.
// User should never use this.
enum status_code readDirect(struct i2c_master_module* i2c_device,
							uint16_t slaveaddr, uint16_t datalen, uint8_t* data){
  struct i2c_master_packet readpacket;
  readpacket.address     = slaveaddr;
  readpacket.data_length = datalen;
  readpacket.data        = data;
  readpacket.ten_bit_address = 0; // These three must be set to zero
  readpacket.hs_master_code = 0;
  readpacket.high_speed = 0;
  return i2c_master_read_packet_wait(i2c_device, &readpacket);
}

// This does a write to set the reading memory location and a read to get the memory.
// Arbitrarily long reads are permitted.
enum status_code getDirect(struct i2c_master_module* i2c_device,
						   uint16_t slaveaddr, uint16_t wdatalen,
						   uint16_t rdatalen, uint8_t* wdata, uint8_t* rdata){
  volatile enum status_code status = writeDirect(i2c_device, slaveaddr, wdatalen, wdata);
  if(status != STATUS_OK) return status; // Check if the write 
  return readDirect(i2c_device, slaveaddr, rdatalen, rdata);
}

int main(void)
{
  system_init();
  delay_init(); // for delay_ms()
  turnon_routine(); // Start VL6180X
  configure_i2c(); // Initializes i2c_master_instance

  volatile int correct_counter = 0; // These are made volatile so that for
									// debugging they aren't optimized out
  volatile int iterations = 0; 
  volatile int errorcount = 0;
  enum status_code getstate; // This will be used so that for debugging
							 // the state of the reads may be verified

  wr_buffer[0] = 0; // Set the write buffer to contain the memory reading address of 0x00.
  wr_buffer[1] = 0;

  while(1){
   iterations++;
    rd_buffer[0] = 0; // Reset value of read buffer for check later
	
    // Read the first 64 bytes of the VL6180X memory:
    getstate = getDirect(&i2c_master_instance, SLAVE_ADDRESS, 2, 64, wr_buffer, rd_buffer);
    if(getstate!=STATUS_OK){ // Check if get went fine, else concatenate the number of errors
      errorcount++;
    }
	
    // If 0 xB4value was read then the read went fine,
    // since 0xB4 is the device ID which should be found at memory location 0x00
    if(rd_buffer[0] == ((uint8_t) 0xB4)) correct_counter += 1; // If this value was read then
															   // the read went fine
    delay_ms(10);
  }
}
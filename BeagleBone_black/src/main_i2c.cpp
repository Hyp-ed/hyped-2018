
/*
 * Authors : M. Kristien, E. van Woerkom
 * Organisation: HYPED
 * Date: 3. February 2018
 * Description: 
 * This is an example of how a main function using I2C would look like
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

#include <stdint.h>
#include "i2c/i2c.hpp"

void initialiseSlaves(i2c::Driver &instance) {
  /* What this does is slave dependent, suggested for VL6180X:
  Turn on/off GPIO pins to boot
  Set registers to continuous operation mode with given frequency
  Get test values to establish working
  */
}

int main(void)
{
  i2c::Driver device1 = i2c::Driver();
  /* Replace with your application code */
  
  // Set up context for driver configuration:
  i2c::Context device1_settings;  // Must be filled appropriately
  
  device1.configure(&device1_settings);     // Performs no I2C operations but will interact with controller hardware
  initialiseSlaves(device1);       // Will perform real I2C operations, will take some time to run to setup VL6180X
  i2c::Channel inputs[10];          // This should be further initialised
  device1.configureChannels(inputs, 10);   // Requires no communications, purely registration in software. 
  
  while (1) 
  {
    device1.control();
    // It is suggested that in the while loop the health
    // of the I2C connection is monitored by checking error counters and statuses
    // Meanwhile, Control() will be automatically running communications
    // and calling callback functions to make aware of new results
  }
}

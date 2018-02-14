/*
 * examplei2cmain.cpp
 *
 * Created: 03/02/2018 11:56:25
 * Author : M. Kristien, E. van Woerkom
 * This is an example of how a main function using I2C would look like
 */ 


#include "sam.h"
#include "i2c/i2c.hpp"

void initialise_slaves(i2c::Driver &instance) {
/* What this does is slave dependent, suggested for VL6180X:
Turn on/off GPIO pins to boot
Set registers to continuous operation mode with given frequency
Get test values to establish working
*/
}

int example_main(void)
{
    /* Initialize the SAM system */
    SystemInit();
  i2c::Driver device1 = i2c::Driver();
    /* Replace with your application code */
  
  // Set up context for driver configuration:
  i2c::Context device1_settings; // Must be filled appropriately
  
  device1.Configure(&device1_settings); // Performs no I2C operations but will interact with controller hardware
  initialise_slaves(device1); // Will perform real I2C operations, will take some time to run to setup VL6180X
  i2c::Channel inputs[10]; // This should be further initialised
  device1.Configure_channels(inputs, 10); // Requires no communications, purely registration in software. 
  
  while (1) 
  {
  device1.Control();
  // It is suggested that in the while loop the health
  // of the I2C connection is monitored by checking error counters and statuses
  // Meanwhile, Control() will be automatically running communications
  // and calling callback functions to make aware of new results
  }
}

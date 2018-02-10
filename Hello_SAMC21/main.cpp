/*
 * Hello_SAMC21.cpp
 *
 * Created: 03/02/2018 11:56:25
 * Author : martin
 */ 


#include "sam.h"
#include "i2c/i2c.hpp"

void initialise_slaves(i2c::Driver &instance) {}
/*
Turn on/off GPIO pins to boot
Set registers to continuous operation mode with given frequency
Get test values to establish working
*/

int main(void)
{
    /* Initialize the SAM system */
    SystemInit();
	i2c::Driver device1 = i2c::Driver();
    /* Replace with your application code */
	
	// Set up context for driver configuration:
	i2c::Context device1_settings; // Must be filled appropriately
	
	device1.Configure(&device1_settings); // Performs no I2C operations but will interact with controller hardware
	initialise_slaves(device1); // Will perform real I2C operations, will take time to run
	i2c::Channel inputs[10]; // This should be further initialised
	device1.Configure_channels(inputs, 10); // Requires no communications, purely registration in software. 
	
    while (1) 
    {
		device1.Control();
		// Checks of how many errors have been, how actual is the information
    }
}

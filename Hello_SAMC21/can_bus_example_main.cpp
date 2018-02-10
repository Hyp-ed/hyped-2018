/*
 * can_bus_example_main.cpp
 *
 * Created: 10.2.2018 21:51:10
 *  Author: E. Elgun
 */ 

#include "sam.h"
#include "can_bus.hpp"

int main(void)
{
	/* Initialize the SAM system */
	SystemInit();
	context *context1;
	CAN can_module1;
	can_module1.configure(&context1);
	can_module1.initialize();
	//active boolean can be changed anytime during the process if needed.
	//Following will make the can_module1 to listen the specified stream. s
	can_module1.rx_module1_recieve.active = true;
	
	/* Replace with your application code */
	while (1)
	{
		//Will handle all transmissions.
		can_module1.control();
	}
}
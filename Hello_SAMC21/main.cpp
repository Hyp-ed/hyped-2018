/*
 * Hello_SAMC21.cpp
 *
 * Created: 03/02/2018 11:56:25
 * Author : martin
 */ 


#include "sam.h"
#include "i2c/i2c.hpp"

int main(void)
{
    /* Initialize the SAM system */
    SystemInit();
	i2c::Driver device1 = i2c::Driver();
    /* Replace with your application code */
    while (1) 
    {
    }
}

/*
 * i2c.h
 *
 * Created: 10/02/2018 10:18:58
 *  Author: kristien
 */ 


#ifndef I2C_HPP_
#define I2C_HPP_

namespace i2c {

// Single i2c message 
struct Packet {
	uint8_t i;
};

// Single instance of R/W with a slave i2c device
struct Channel {
	
};

// Configuration context
struct Context {
	
};


class Driver {
public:
	void Configure();
};
	
}


#endif /* I2C_HPP_ */
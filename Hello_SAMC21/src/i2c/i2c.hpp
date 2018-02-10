/*
 * i2c.hpp
 *
 * Created: 10/02/2018 10:18:58
 * Authors: M. Kristien, E. van Woerkom
 */ 


#ifndef I2C_HPP_
#define I2C_HPP_

namespace i2c {

struct Channel;
struct Context;
class Driver;

// Single instance of R/W periodic communications channel with a slave i2c device
struct Channel {
	uint8_t address;
	char direction;
	// 3 direction possibilities: W(rite), R(ead), G(et),
	// with get being a write-read operation to retrieve register data,
	// read is most likely never used, get often.
	char* readbuffer, writebuffer; // Pointers to be left null if no read or write takes place
	int readbuflength, writebuflength;
	int period; // Number of microseconds to wait to start a new reading
	int* status; // If something goes wrong, this must be set using || (or) so as not to remove unreseted errors
	int* error_counter, success_counter; // To be incremented after every failure and success
	void *callbackfunction(Channel* current_channel, Driver current_device);
	// The above callbackfunction is called after every operation in this channel
};

// Configuration context
struct Context {
	// This is completely hardware dependent
	// It will contain things such as physical port names to be used,
	// the responsible interfaces and everything necessary to instantiate
	// a working I2C master.
};


class Driver {
public:
	// Init Functions:
	void Configure(Context* ctx){} // Set up hardware controller parameters, make connection ready, copy ctx into here
	void Configure_channels(Channel *channels, int len){} // Register continuous periodic communication channels
	
	void Control(){} // Called in while loop to update devices and cycle channel communications
	
	// DirectWrite does a blocking write operation. It writes data from the writebuffer and returns
	// a status. This is to be defined but suggested 0 for success, -1 for writing failure and -2 for bus busy.
	// This function is not to be used by end-user but most likely in setup phase in initialise_slaves()
	int DirectWrite(uint8_t addr, char* writebuffer, int len) { return 0; }
	// DirectRead is similarly defined and has same behaviour
	int DirectRead(uint8_t addr, char* readbuffer, int len) { return 0; }
private:
	Context* ctx_; // Contains device settings
	Channel* activated_channels; // Pointer to array with all channels to be used
	int channel_number; // Number of elements in activated_channels
	void do_communication(Channel* message); // This function will execute a single communication as specified
	// Furthermore it will increment counters, set appropriate error bytes, call callbackfunction
	void cycle_channels(); // This will cycle through all current activated channels
	// with do_communication and run if necessary.
};
	
}


#endif /* I2C_HPP_ */
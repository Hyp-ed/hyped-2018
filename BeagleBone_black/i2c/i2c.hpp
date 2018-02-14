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
  /**
   * @brief      Configure library with channel-agnostic data. Setup hardware
   *             controller parameters. Make I2C hardware ready.
   *
   * @param      ctx   Configuration context
   */
  void Configure(Context* ctx) { ctx_ = ctx; }

  /**
   * @brief      Register continuous periodic communication channels.
   *
   * @param      channels  pointer to an array of communication channels
   * @param[in]  len       number of channels in the array
   */
  void Configure_channels(Channel *channels, int len) {
    activated_channels_ = channels;
    channel_number_      = len;
  }
  
  /**
   * @brief      Periodically called routine to handle channel manipulations
   */
  void Control() { /* EMPTY */ } 
  
  /**
   * @brief      Blocking write operation. Can be used to initialise slave devices.
   *
   * @param[in]  addr         I2C slave address
   * @param      writebuffer  data buffer pointer
   * @param[in]  len          number of bytes to write
   *
   * @return     success status
   */
  int DirectWrite(uint8_t addr, char* writebuffer, int len) { return 0; }
  
  /**
   * @brief      Blocking read operation. Can be used to initialise slave devices.
   *
   * @param[in]  addr         I2C slave address
   * @param      writebuffer  data buffer pointer
   * @param[in]  len          number of bytes to read
   *
   * @return     success status
   */
  int DirectRead(uint8_t addr, char* readbuffer, int len) { return 0; }

 private:
  /**
   * @brief      Handle communication of I2C packet corresponding to one channel.
   *
   * @param      message  pointer to the corresponding channel
   */
  void do_communication(Channel* message);

// -----------------------------------------------------------------------------
 private:
  Context* ctx_;
  Channel* activated_channels_;
  int channel_number_;
};
  
}   // namespace i2c


#endif /* I2C_HPP_ */
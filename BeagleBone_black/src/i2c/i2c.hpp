
/*
 * Authors: M. Kristien, E. van Woerkom
 * Organisation: HYPED
 * Date: 10. February 2018
 * Description:
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


#ifndef BEAGLEBONE_BLACK_I2C_I2C_HPP_ 
#define BEAGLEBONE_BLACK_I2C_I2C_HPP_ 

namespace hyped::i2c {

struct Channel;
struct Context;
class Driver;

// Single instance of R/W periodic communications channel with a slave i2c dev
struct Channel {
  uint8_t address;
  char direction;
  // 3 direction possibilities: W(rite), R(ead), G(et),
  // with get being a write-read operation to retrieve register data,
  // read is most likely never used, get often.
  char* readbuffer;
  char* writebuffer;
  int readbuflength;
  int writebuflength;
  int period;   // Number of microseconds to wait to start a new reading
  int* status;
  int* error_counter;
  int* success_counter;
  void (*callback)(Channel* current_channel, Driver current_device);
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
  void configure(Context* ctx) { ctx_ = ctx; }

  /**
   * @brief      Register continuous periodic communication channels.
   *
   * @param      channels  pointer to an array of communication channels
   * @param[in]  len       number of channels in the array
   */
  void configureChannels(Channel *channels, int len) {
    activated_channels_ = channels;
    channel_number_      = len;
  }
  
  /**
   * @brief      Periodically called routine to handle channel manipulations
   */
  void control() { /* EMPTY */ } 
  
  /**
   * @brief      Blocking write operation. Can be used to initialise slave devices.
   *
   * @param[in]  addr         I2C slave address
   * @param      writebuffer  data buffer pointer
   * @param[in]  len          number of bytes to write
   *
   * @return     success status
   */
  int directWrite(uint8_t addr, char* writebuffer, int len) { return 0; }
  
  /**
   * @brief      Blocking read operation. Can be used to initialise slave devices.
   *
   * @param[in]  addr         I2C slave address
   * @param      writebuffer  data buffer pointer
   * @param[in]  len          number of bytes to read
   *
   * @return     success status
   */
  int directRead(uint8_t addr, char* readbuffer, int len) { return 0; }

 private:
  /**
   * @brief      Handle communication of I2C packet corresponding to one channel.
   *
   * @param      message  pointer to the corresponding channel
   */
  void doCommunication(Channel* message);

  Context* ctx_;
  Channel* activated_channels_;
  int channel_number_;
};
  
}   // namespace hyped::i2c


#endif  /* BEAGLEBONE_BLACK_I2C_I2C_HPP_  */
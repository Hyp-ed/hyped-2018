/*
 * Authors : Ege Elgun
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

 #include "utils/io/i2c.hpp"

using hyped::utils::io::I2C;
using hyped::utils::Logger;
using hyped::utils::concurrent::Thread;

Logger log(true,1);

int main(){
    /**
     * Creating an I2C object. Arguments are lane number to whcich sensor
     * is connected to, address of the device on I2C lane, and 10bit adressing (1 for on,
     * 0 for off). Current version does not support multiple slaves on the same lane.
    **/
	I2C* i1 = new I2C(2, 0x29, 0); //Initialize the i2c lane and connect to device.
    
    Char* buf_read[72];              //Create an array for read. 
    Char* buf_write[72] = {1};       //Create array for write and assign value.
    i1->i2c_read(buf_read, 72);      //Read from the device for 82 bytes in to buf_read.

    //Write 72 bytes from write_buf to device. No registry adressing at this point.
    i1->i2c_write(buf_read, 72);

    delete i1;

	return 0;
}

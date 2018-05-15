
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

#include "utils/io/i2c.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"

using hyped::utils::io::I2C;
using hyped::utils::Logger;

Logger log(true, 1);

int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
	I2C& i2c = I2C::getInstance();
	uint8_t tx[] = {0, 0};
	uint8_t rx[8] = {};

	i2c.write(0x29, tx, 2);
	i2c.read(0x29, rx, 8);
	log.INFO("MAIN", "read %x %x %x %x", rx[0], rx[1], rx[2], rx[3]);
	return 0;
}

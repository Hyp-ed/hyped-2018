
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

#include <iostream>
#include "utils/io/i2c/i2c.hpp"

using hyped::utils::io::I2C;
using namespace std;

Logger log(true,1);

int main(){
	Thread* t1 = new I2C();

	t1->start();
	t1->join();
	cout << "I2C Thread joined and executed" << endl;
	return 0;
}

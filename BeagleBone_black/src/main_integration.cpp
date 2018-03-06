/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 
 * Description: <What is this file for?>
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *    except in compliance with the License. You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software distributed under
 *    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *    either express or implied. See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include <iostream>
#include "data/data_point.hpp"
#include "utils/math/integration.hpp"

hyped::data::DataPoint<double> a(1,1);
hyped::data::DataPoint<double> b(2,1);
hyped::data::DataPoint<double> c(3,1);
hyped::data::DataPoint<double> d(4,2);
hyped::data::DataPoint<double> e(5,5);

int main()
{
  hyped::utils::math::Integration<double> area;
  std::cout << area.update(a,b) << std::endl;
  std::cout << area.update(b,c) << std::endl;
  std::cout << area.update(c,d) << std::endl;
  std::cout << area.update(d,e) << std::endl;
}


/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 24 February 2018
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

#include <iostream>

#include "utils/math/kalman.hpp"
#include "utils/math/vector.hpp"

using hyped::utils::math::Kalman;
using hyped::utils::math::Vector;

int main()
{
  double value, p, q = 0.1;
  int total;

  std::cin >> total >> p;
  Kalman<double> kalman(0, p, q);

  for (int i = 0; i < total; i++) {
    std::cin >> value;
  }

  for (int i = 0; i < total; i++) {
    std::cin >> value;
    std::cout << kalman.filter(value) << '\n';
  }
}


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

template <typename T, int dimension>
void print(const Vector<T, dimension>& vector)
{
  for (int i = 0; i < dimension; i++)
    std::cout << vector[i] << '\t';
  std::cout << '\n';
}

int main()
{
  Kalman<Vector<double, 1>> kalman(Vector<double, 1>(0), Vector<double, 1>(1),
                                   Vector<double, 1>(1));

  for (int i = 0; i < 100; i++)
    print(kalman.filter(Vector<double, 1>(i)));
  std::cout << '\n';
}

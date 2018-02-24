
/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 24 February 2018
 * Description:
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License,  Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing,  software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,  either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include <iostream>
#include "utils/math/vector.hpp"

using hyped::utils::math::Vector;

template <typename T,  int N>
void print(Vector<T,  N> &v)
{
  for (int i = 0; i < N; i++)
    std::cout << v[i] << '\t';
  std::cout << '\n';
}

int main()
{
  Vector<int, 10> vector1;
  Vector<int, 10> vector2;
  Vector<int, 10> vector3;
  Vector<double, 10> vector4;
  Vector<double, 2> vector5({3, 4});
  Vector<int, 2> vector6(1);
  Vector<double, 2> vector7(1.0/sqrt(2));

  print(vector1);
  print(vector2);
  print(vector3);

  for (int i = 0; i < 10; i++)
    vector1[i] = i+1,
      vector4[i] = 1.1;

  print(vector4);

  vector2 += 1;
  vector3 = vector1 + vector2;

  vector2 = vector2 - 2;
  vector3 = vector1 - vector2;

  vector3 = vector1 * 5.0;
  vector3 = vector1 / 5.0;

  vector4 = vector4 + vector1;
  vector4 = vector4 - vector1;

  (vector1 == vector3) ? std::cout << "False\n" : std::cout << "True\n";

  (vector5.norm() == 5) ? std::cout << "True\n" : std::cout << "False\n";

  (vector6.norm() == sqrt(2)) ? std::cout << "True\n" : std::cout << "False\n";

  (vector6.toUnitVector() == vector7) ? std::cout << "True\n" : std::cout << "False\n";

  print(vector1);
  print(vector2);
  print(vector3);
  print(vector4);
}

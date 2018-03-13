
/*
 * Author: Adithya Sireesh
 * Organisation: HYPED
 * Date: 3 March 2018
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
#include "utils/math/quaternion.hpp"

using hyped::utils::math::Quaternion;

template <typename T>
void print(Quaternion<T> &q)
{
  char a[4] = {' ', 'i', 'j', 'k'};
  char op[4] = {'+', '+', '+', ' '};
  for (int i = 0; i < 4; i++)
    std::cout << q[i] << a[i] << ' ' << op[i] << ' ';
  std::cout << '\n';
}

int main()
{
  Quaternion<int> quaternion1;
  Quaternion<int> quaternion2;
  Quaternion<int> quaternion3;
  Quaternion<double> quaternion4;
  Quaternion<double> quaternion5(11, 4, 4, 4);
  Quaternion<int> quaternion6(1);
  Vector<int, 3> vector7({2, 3, 4});
  Quaternion<double> quaternion7(vector7);
  Quaternion<double> quaternion8;
  Quaternion<double> quaternion9(-25, 30, 29, 52);
  Quaternion<double> quaternion10(5, vector7);
  Quaternion<int> quaternion11(quaternion10);

  print(quaternion1);
  print(quaternion2);
  print(quaternion3);

  for (int i = 0; i < 4; i++)
    quaternion1[i] = i+1,
      quaternion4[i] = 1.1;

  print(quaternion4);

  quaternion2 += 1;
  quaternion3 = quaternion1 + quaternion2;

  quaternion2 = quaternion2 - 2;
  quaternion3 = quaternion1 - quaternion2;

  quaternion3 = quaternion1 * 5.0;
  quaternion3 = quaternion1 / 5.0;

  quaternion4 = quaternion4 + quaternion1;
  quaternion4 = quaternion4 - quaternion1;

  quaternion8 = quaternion5*quaternion1;

  quaternion10-=quaternion11;
  quaternion10+=quaternion11;

  (quaternion1 == quaternion3) ? std::cout << "False\n" : std::cout << "True\n";

  (quaternion5.norm() == 13) ? std::cout << "True\n" : std::cout << "False\n";

  (quaternion6.norm() == 2) ? std::cout << "True\n" : std::cout << "False\n";

  (quaternion8 == quaternion9) ? std::cout << "True\n" : std::cout << "False\n";

  (quaternion10 == quaternion11) ? std::cout << "True\n" : std::cout << "False\n";


  // TODO(Adi): Check if this function is required
  // (quaternion6.toUnitQuaternion() == Quaternion7) ?
  // std::cout << "True\n" : std::cout << "False\n";

  print(quaternion1);
  print(quaternion2);
  print(quaternion3);
  print(quaternion4);
  print(quaternion5);
  print(quaternion6);
  print(quaternion7);
  print(quaternion8);
  print(quaternion9);
  print(quaternion10);
  print(quaternion11);
}

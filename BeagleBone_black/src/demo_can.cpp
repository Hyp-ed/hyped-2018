/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 14. March 2018
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

#include <unistd.h>

#include "utils/io/can.hpp"

using hyped::utils::io::Can;
using hyped::utils::io::CanFrame;


inline void delay(int ms)
{
  usleep(ms * 1000);
}

int main()
{
  CanFrame data = {14, 4, {1, 2, 3, 4, 5, 6, 7, 8}};

  Can can(0);
  can.send(data);
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < data.len; j++) {
      data.data[j] += 1;
    }
    data.id += 1;
    can.send(data);
  }

  delay(10);
}


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
#include <sstream>
#include <fstream>

#include "utils/math/kalman.hpp"
#include "utils/math/vector.hpp"

using hyped::utils::math::Kalman;
using hyped::utils::math::Vector;

Vector<double, 3> mysqrt(Vector<double, 3> x) {
  Vector<double, 3> temp(0);

  temp[0] = sqrt(x[0]);
  temp[1] = sqrt(x[1]);
  temp[2] = sqrt(x[2]);

  return temp;
}

int main()
{
  for (double q_i = 0.1; q_i < 5.0; q_i += 0.1) {
    for (int file_number = 0; file_number < 4; file_number++) {
      std::cout << "Performing filter for file number and process noise: " << file_number << ' ' << q_i << '\n';

      std::string temp;
      int counter = 0;
      Vector<double, 3> acc_sum_sq(0), acc_sum(0);
      Vector<double, 3> gyr_sum_sq(0), gyr_sum(0);
      long long timestamp = 0, prev_timestamp = -1;
      Vector<double, 3> acc(0), gyr(0), p_acc(0), q_acc(q_i), 
                        p_gyr(0), q_gyr(q_i), f_acc(0), f_gyr(0);
      Kalman<Vector<double, 3>> k_acc, k_gyr;

      // Read file
      std::ostringstream r_strs;
      r_strs << "./data/test/imu_" << file_number << "_test.txt";
      std::string r_file_name = r_strs.str();
      std::ifstream r_file;
      r_file.open(r_file_name);

      // Write file
      std::ostringstream w_strs;
      w_strs << "./data/test/filtered/noise_" << q_i << "_imu_" << file_number << ".txt";
      std::string w_file_name = w_strs.str();
      std::ofstream w_file;
      w_file.open(w_file_name);

      for (;;) {
        r_file >> timestamp >> acc[0] >> acc[1] >> acc[2] >> gyr[0] >> gyr[1] >> gyr[2] >> temp;
        if (timestamp == prev_timestamp) {
          break;
        }
        prev_timestamp = timestamp;

        if (counter < 1000) {
          acc_sum += 0.001*acc;
          acc_sum_sq += (0.001*acc)*acc;

          gyr_sum += 0.001*gyr;
          gyr_sum_sq += (0.001*gyr)*gyr;

          if (counter == 999) {
            p_acc = mysqrt(acc_sum_sq - (acc_sum*acc_sum));
            p_gyr = mysqrt(gyr_sum_sq - (gyr_sum*gyr_sum));

            k_acc.configure(acc, p_acc, q_acc);
            k_gyr.configure(gyr, p_gyr, q_gyr);
          }

          counter ++;
          continue;
        }

        f_acc = k_acc.filter(acc);
        f_gyr = k_gyr.filter(gyr);

        w_file << timestamp;
        for (int i = 0; i < 3; i++) {
          w_file << " " << f_acc[i];
        } 

        for (int i = 0; i < 3; i++) {
          w_file << " " << f_gyr[i];
        }

        w_file << '\n';
      }

      w_file.close();
      r_file.close();
      std::cout << "Completed. Moving to the next\n";
    }
  }
}

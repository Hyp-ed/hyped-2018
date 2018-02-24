
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

/*
  TODO: Decide the process noise value
*/

#ifndef BEAGLEBONE_BLACK_UTILS_MATH_KALMAN_HPP_
#define BEAGLEBONE_BLACK_UTILS_MATH_KALMAN_HPP_

namespace hyped {
namespace utils {
namespace math {

template<typename T>
class Kalman {
 public:
  Kalman(T input_value, T measurement_noise, T estimation_error = 0)
    : measurement_noise_covariance_(measurement_noise),
    estimation_error_covariance_(estimation_error),
    filtered_value_(input_value)
    {}

  T filter(T input)
  {
    estimation_error_covariance_ += k_process_noise_;
    kalman_gain_ = estimation_error_covariance_ /
      (estimation_error_covariance_ + measurement_noise_covariance_);
    filtered_value_ += kalman_gain_ * (input - filtered_value_);
    estimation_error_covariance_ = (1 - kalman_gain_) * estimation_error_covariance_;

    return filtered_value_;
  }

 private:
  const T k_process_noise_ = 0.01;

  T measurement_noise_covariance_;
  T estimation_error_covariance_;
  T kalman_gain_;
  T filtered_value_;
};
}}}

#endif  // BEAGLEBONE_BLACK_UTILS_MATH_KALMAN_HPP_

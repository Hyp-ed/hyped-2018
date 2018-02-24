
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

#include "utils/math/vector.hpp"

#ifndef BEAGLEBONE_BLACK_UTILS_MATH_KALMAN_HPP_
#define BEAGLEBONE_BLACK_UTILS_MATH_KALMAN_HPP_

namespace hyped {
namespace utils {
namespace math {

template<typename T, int dimension>
class Kalman {
 public:
  Kalman(Vector<T, dimension> input_value, Vector<T, dimension> measurement_noise,
         Vector<T, dimension> process_noise,
         Vector<T, dimension> estimation_error = Vector<T, dimension>())
    : measurement_noise_covariance_(measurement_noise),
      estimation_error_covariance_(estimation_error),
      filtered_value_(input_value),
      process_noise_(process_noise)
    {}

  Vector<T, dimension>& filter(const Vector<T, dimension>& input)
  {
    estimation_error_covariance_ += process_noise_;
    kalman_gain_ = estimation_error_covariance_ /
      (estimation_error_covariance_ + measurement_noise_covariance_);
    filtered_value_ += kalman_gain_ * (input - filtered_value_);
    estimation_error_covariance_ = (1 - kalman_gain_) * estimation_error_covariance_;

    return filtered_value_;
  }

  Vector<T, dimension> getFiltered()
  {
    return filtered_value_;
  }

 private:
  Vector<T, dimension> process_noise_;
  Vector<T, dimension> measurement_noise_covariance_;
  Vector<T, dimension> estimation_error_covariance_;
  Vector<T, dimension> kalman_gain_;
  Vector<T, dimension> filtered_value_;
};
}}}

#endif  // BEAGLEBONE_BLACK_UTILS_MATH_KALMAN_HPP_

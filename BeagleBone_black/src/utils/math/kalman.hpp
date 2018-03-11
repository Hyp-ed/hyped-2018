
/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 24 February 2018
 * Description:
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

// TODO(Uday): Decide the process noise value

#include "utils/math/vector.hpp"

#ifndef BEAGLEBONE_BLACK_UTILS_MATH_KALMAN_HPP_
#define BEAGLEBONE_BLACK_UTILS_MATH_KALMAN_HPP_

namespace hyped {
namespace utils {
namespace math {

// TODO(Uday): Fill in the class comment.
// TODO(Uday): Explain a little bit about Kalman filters in class comment or file comment
/**
 * @brief
 *
 * @tparam T
 */
template <typename T>
class Kalman {
 public:
  // TODO(Uday): Fill in the method comments.
  /**
   * @brief Construct a new Kalman object
   *
   * @param input_value
   * @param measurement_noise
   * @param process_noise
   * @param estimation_error
   */
  Kalman(T input_value, T measurement_noise, T process_noise, T estimation_error = T());

  /**
   * @brief
   *
   * @param input
   * @return T
   */
  T filter(const T& input);

  /**
   * @brief Get the Filtered object
   *
   * @return T
   */
  T getFiltered();

 private:
  T process_noise_;
  T measurement_noise_covariance_;
  T estimation_error_covariance_;
  T kalman_gain_;
  T filtered_value_;
};

template <typename T>
Kalman<T>::Kalman(T input_value, T measurement_noise, T process_noise, T estimation_error)
    : process_noise_(process_noise),
      measurement_noise_covariance_(measurement_noise),
      estimation_error_covariance_(estimation_error),
      filtered_value_(input_value)
{}

template <typename T>
T Kalman<T>::filter(const T& input)
{
  estimation_error_covariance_ += process_noise_;
  kalman_gain_ = estimation_error_covariance_ /
                (estimation_error_covariance_ + measurement_noise_covariance_);
  filtered_value_ += kalman_gain_ * (input - filtered_value_);
  estimation_error_covariance_ = (1 - kalman_gain_) * estimation_error_covariance_;

  return filtered_value_;
}

template <typename T>
T Kalman<T>::getFiltered()
{
  return filtered_value_;
}

}}}  // hyped::util::math

#endif  // BEAGLEBONE_BLACK_UTILS_MATH_KALMAN_HPP_

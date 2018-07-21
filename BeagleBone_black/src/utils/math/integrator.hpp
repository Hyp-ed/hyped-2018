/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 6 March 2018
 * Description: Integration class to calculate the area of the
 *              graph between two points.
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

#include "data/data_point.hpp"

#ifndef BEAGLEBONE_BLACK_UTILS_MATH_INTEGRATOR_HPP_
#define BEAGLEBONE_BLACK_UTILS_MATH_INTEGRATOR_HPP_

namespace hyped {
namespace utils {
namespace math {

using hyped::data::DataPoint;

/**
 * @brief Calculates an integral. Supports shared output variables that are modifiable by other code
 *        as well.
 *
 * @tparam T Underlying numeric type
 */
template <typename T>
class Integrator {
 public:
  /**
   * @brief Construct a new Integrator object
   *
   * @param output Where the result is stored. Caller is responsible for freeing the memory
   */
  explicit Integrator(DataPoint<T>* output);

  /**
   * @brief    Calculates the area given two points for time T_(N)
   *
   * @param[in]  point1    The point for time T_(N-2)
   * @param[in]  point2    The point for time T_(N-1)
   */
  DataPoint<T> update(const DataPoint<T>& point);

 private:
  DataPoint<T> previous_point_;
  DataPoint<T>* output_;
  bool initialised_;
};

template <typename T>
Integrator<T>::Integrator(DataPoint<T>* output)
    : previous_point_(0, T(0)), output_(output), initialised_(false)
{}

template <typename T>
DataPoint<T> Integrator<T>::update(const DataPoint<T>& point)
{
  if (!initialised_) {
    previous_point_ = point;
    initialised_ = true;
  }
  // TODO(Brano,anyone): Change timestamp in DataPoint to use std::chrono::duration
  // Assume timestamp in microseconds and convert to seconds
  T area = (point.value + previous_point_.value)/2 *
           ((point.timestamp - previous_point_.timestamp)/1e6);

  output_->value += area;
  output_->timestamp = point.timestamp;

  previous_point_ = point;

  return *output_;
}

}}}  // hyped::utils::math

#endif  // BEAGLEBONE_BLACK_UTILS_MATH_INTEGRATOR_HPP_

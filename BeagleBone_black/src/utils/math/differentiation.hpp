/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 6 March 2018
 * Description: Differentiation class to calculate the gradient of
 *              the graph between two points.
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

#ifndef BEAGLEBONE_BLACK_UTILS_MATH_DIFFERENTIATION_HPP_
#define BEAGLEBONE_BLACK_UTILS_MATH_DIFFERENTIATION_HPP_

namespace hyped {
namespace utils {
namespace math {

using namespace hyped::data;
  
template <typename T>
class Differentiation {
 public:
  Differentiation();

  /**
   * @brief    Calculates the gradient given two points for time T_(N)
   *
   * @param[in]  point1    The point for time T_(N-2)
   * @param[in]  point2    The point for time T_(N-1)
   *
   */
  DataPoint<T> update(DataPoint<T> point);

 private:
  DataPoint<T> prev_point_;
  
};

template <typename T>
Differentiation<T>::Differentiation()
{
  prev_point_ = DataPoint<T>(0,0);
}
  
template <typename T>
DataPoint<T> Differentiation<T>::update(DataPoint<T> point)
{
  T gradient = (point.value - prev_point_.value) / (point.timestamp - prev_point_.timestamp);
  
  prev_point_.value = point.value;
  prev_point_.timestamp = point.timestamp;

  return DataPoint<T>(point.timestamp, gradient);
}

}}}

#endif  // BEAGLEBONE_BLACK_UTILS_MATH_DIFFERENTIATION_HPP_

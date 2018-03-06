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

template <typename T>
class Differentiation {
 public:
  Differentiation() {}

  /**
   * @brief    Calculates the gradient given two points for time T_(N)
   *
   * @param[in]  point1    The point for time T_(N-2)
   * @param[in]  point2    The point for time T_(N-1)
   *
   */
  hyped::data::DataPoint<T> update(hyped::data::DataPoint<T> point1,
				   hyped::data::DataPoint<T> point2);
  
};

template <typename T>
hyped::data::DataPoint<T> Differentiation<T>::update(hyped::data::DataPoint<T> point1,
						     hyped::data::DataPoint<T> point2)
{
  T gradient = (point2.value - point1.value) / (point2.timestamp - point1.timestamp);
  return hyped::data::DataPoint<T>(point2.timestamp, gradient);
}

}}}

#endif  // BEAGLEBONE_BLACK_UTILS_MATH_DIFFERENTIATION_HPP_

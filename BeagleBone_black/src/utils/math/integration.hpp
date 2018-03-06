/*
 * Author: Uday Patel
 * Organisation: HYPED
 * Date: 
 * Description: <What is this file for?>
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

#ifndef BEAGLEBONE_BLACK_UTILS_MATH_INTEGRATION_HPP_
#define BEAGLEBONE_BLACK_UTILS_MATH_INTEGRATION_HPP_

namespace hyped {
namespace utils {
namespace math {

template <typename T>
class Integration {
 public:
  Integration() {}

  T update(hyped::data::DataPoint<T> point1,
	   hyped::data::DataPoint<T> point2);

 private:
  T previous_output_;
  
};

template <typename T>
T Integration<T>::update(hyped::data::DataPoint<T> point1,
			 hyped::data::DataPoint<T> point2)
{
  T area = (point2.value + point1.value)/2 * (point2.timestamp - point1.timestamp);
  previous_output_ += area;
  return previous_output_;
}

}}}

#endif  // BEAGLEBONE_BLACK_UTILS_MATH_INTEGRATION_HPP_

/*
 * Author: Brano, Adi, Uday
 * Organisation: HYPED
 * Date: 18 March 2018
 * Description: Main file for navigation class.
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

#include "navigation/main.hpp"

namespace hyped {
namespace navigation {

Main::Main(uint8_t id) : Thread(id), data_(data::Data::getInstance())
{/* EMPTY */}

void Main::run()
{
  while (1) {
    // TODO(Brano,Uday,Adi): Put code.
  }
}

}}  // namespace hyped::navigation

/*
 * Authors: Kofi and Isabella
 * Organisation: HYPED
 * Date: 15/04/18
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

#include "communications/sender.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;

namespace communications {

SenderThread::SenderThread(Communications* baseCommunicator)
    : Thread()
    , baseCommunicator(baseCommunicator)
{ /* Empty */ }

void SenderThread::run()
{
  while (1) {
    nav_ = data.getNavigationData();
    mtr_ = data.getMotorData();
    baseCommunicator->sendDistance(nav_.distance);
    baseCommunicator->sendVelocity(nav_.velocity);
    baseCommunicator->sendAcceleration(nav_.acceleration);
    // baseCommunicator->sendStripeCount(nav.stripe_count);
    baseCommunicator->sendRpmFl(mtr_.rpm_FL);
    baseCommunicator->sendRpmFr(mtr_.rpm_FR);
    baseCommunicator->sendRpmBl(mtr_.rpm_BL);
    baseCommunicator->sendRpmBr(mtr_.rpm_BR);
  }
}

}}

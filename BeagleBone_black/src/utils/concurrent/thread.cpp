/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 21. February 2018
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

#include "utils/concurrent/thread.hpp"

namespace hyped {
namespace utils {
namespace concurrent {

namespace {

void thread_entry_point(Thread* this_)
{
  this_->run();
}

}   // namespace ::


Thread::Thread(uint8_t id, Logger& log)
    : id_(id)
    , thread_(0)
    , log_(log)
{ /* EMPTY */ }

Thread::~Thread() { /* EMPTY */ }

void Thread::start()
{
  thread_ = new std::thread(thread_entry_point, this);
}

void Thread::join()
{
  thread_->join();
}

void Thread::run()
{
  log_.INFO("[THREAD]: You are starting EMPTY thread. Terminating now.\n");
}

void Thread::yield()
{
  std::this_thread::yield();
}

}}}   // namespace hyped::utils::concurrent


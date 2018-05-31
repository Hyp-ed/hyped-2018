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

#include <chrono>

#include "utils/system.hpp"

namespace hyped {
namespace utils {
namespace concurrent {

namespace {

void thread_entry_point(Thread* this_)
{
  this_->run();
}

}   // namespace ::

Thread::Thread(Logger& log)
    : id_(-1),
      thread_(0),
      log_(log)
{ /* EMPTY */ }

Thread::Thread(uint8_t id)
    : id_(id),
      thread_(0),
      log_(System::getLogger())
{ /* EMPTY */ }

Thread::Thread()
    : id_(-1),
      thread_(0),
      log_(System::getLogger())
{ /* EMPTY */ }

Thread::Thread(uint8_t id, Logger& log)
    : id_(id),
      thread_(0),
      log_(log)
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
  log_.INFO("THREAD", "You are starting EMPTY thread. Terminating now.");
}

void Thread::yield()
{
  std::this_thread::yield();
}

void Thread::sleep(uint32_t ms)
{
  std::this_thread::sleep_for(std::chrono::microseconds(ms*1000));
}

void BusyThread::run()
{
  uint64_t i;
  while (running_) {
    i++;
    if (i%10000 == 0) {
      // log_.INFO("BUSY", "output\n");
    }
  }
}

BusyThread::~BusyThread()
{
  running_ = false;
}

}}}   // namespace hyped::utils::concurrent


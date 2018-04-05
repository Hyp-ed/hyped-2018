
/*
 * Authors : HYPED
 * Organisation: HYPED
 * Date: 3. February 2018
 * Description:
 * This is the main executable for BeagleBone pod node
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

// #include <iostream>
#include <stdio.h>

#include "utils/concurrent/thread.hpp"
#include "utils/concurrent/lock.hpp"
#include "utils/concurrent/condition_variable.hpp"

using hyped::utils::concurrent::Thread;
using hyped::utils::concurrent::Lock;
using hyped::utils::concurrent::ScopedLock;
using hyped::utils::concurrent::ConditionVariable;

using hyped::utils::Logger;

ConditionVariable cv;
Lock              global_lock;
Lock              lock_;
Logger            log(true, 1);

int value = 0;

void delay(uint32_t i)
{
  while (i--);
}

class DemoThread: public Thread {
 public:
  explicit DemoThread(uint8_t id, bool synchronise = false)
      : Thread(id, log),
        synchronise_(synchronise)
  { /* EMPTY */ }

  void run() override
  {
    global_lock.lock();
    cv.wait(&global_lock);
    global_lock.unlock();

    if (synchronise_) lock_.lock();
    int temp = value;

    delay(10000);
    value = temp + 1;
    if (synchronise_) lock_.unlock();
  }

 private:
  bool synchronise_;
};


int main()
{
  bool synchronise = false;
  Thread* t2 = new DemoThread(2, synchronise);
  Thread* t3 = new DemoThread(3, synchronise);

  t2->start();
  t3->start();

  delay(100000);

  cv.notifyAll();

  delay(40000);
  int temp = value;

  t2->join();
  t3->join();
  delete t2;
  delete t3;
  printf("synchronise: %d ", synchronise);
  printf("temp vs final value: %d %d\n", temp, value);
  return 0;
}

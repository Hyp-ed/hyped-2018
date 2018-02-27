
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

#include "utils/concurrent/Thread.hpp"
#include "utils/concurrent/Lock.hpp"
#include "utils/concurrent/ConditionVariable.hpp"

using hyped::utils::concurrent::Thread;
using hyped::utils::concurrent::Lock;
using hyped::utils::concurrent::ScopedLock;
using hyped::utils::concurrent::ConditionVariable;

ConditionVariable cv;
Lock              global_lock;
Lock              lock_;

int value = 0;

class DemoThread: public Thread {
 public:
  explicit DemoThread(uint8_t id, bool synchronise = false)
    : Thread(id),
      synchronise_(synchronise) { /* EMPTY */ }
  void run() override
  {
    int i;

    global_lock.lock();
    cv.wait(&global_lock);
    global_lock.unlock();

    if (synchronise_) lock_.lock();
    int temp = value;
    i = 10000;
    while (i--);
    value = temp + 1;
    if (synchronise_) lock_.unlock();
  }
 private:
  bool synchronise_;
};


int main()
{
  int i;
  bool synchronise = false;
  Thread* t2 = new DemoThread(2, synchronise);
  Thread* t3 = new DemoThread(3, synchronise);

  t2->start();
  t3->start();

  i = 100000;
  while (i--);

  cv.notifyAll();

  i = 40000;
  while (i--);
  int temp = value;

  t2->join();
  t3->join();
  delete t2;
  delete t3;
  printf("synchronise: %d ", synchronise);
  printf("temp vs final value: %d %d\n", temp, value);
  return 0;
}

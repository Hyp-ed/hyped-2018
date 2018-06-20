

#include "utils/timer.hpp"
#include "utils/logger.hpp"
#include "utils/concurrent/thread.hpp"
using hyped::utils::Timer;
using hyped::utils::concurrent::Thread;

int main(int argc, char** argv) {
  hyped::utils::Logger log(true, 0);

  while (1) {
    log.INFO("T", "current time %llu %llu", Timer::getTimeMicros(), Timer::getTimeMicros()/1000);
    Thread::sleep(250);
  }
}


#include <stdio.h>
#include "utils/concurrent/thread.hpp"
#include "utils/io/gpio.hpp"
#include "utils/system.hpp"

using hyped::utils::concurrent::Thread;
using hyped::utils::io::GPIO;
using hyped::utils::System;
namespace io = hyped::utils::io;


int main(int argc, char* argv[]) {
  System::parseArgs(argc, argv);

  GPIO pin_66(66, io::gpio::kOut);  // P8_7
  GPIO pin_69(69, io::gpio::kIn);   // P8_9

  for (int i = 0; i < 5; i++) {
    uint8_t val = pin_69.wait();
    printf("gpio changed to %d\n", val);
    // pin_69.read();
    // Thread::sleep(200);
  }

  pin_66.set();
  Thread::sleep(5000);
  pin_66.clear();
  Thread::sleep(5000);
  pin_66.set();
  Thread::sleep(5000);
}
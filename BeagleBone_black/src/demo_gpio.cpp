

#include "utils/io/gpio.hpp"
#include "utils/system.hpp"

using hyped::utils::io::GPIO;
using hyped::utils::System;
namespace io = hyped::utils::io;

int main(int argc, char* argv[]) {
  System::parseArgs(argc, argv);

  GPIO pin_66(66, io::gpio::kOut);

  pin_66.set();
}
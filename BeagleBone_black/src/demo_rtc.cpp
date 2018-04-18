

#include "utils/io/spi.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"

using hyped::utils::io::SPI;
using hyped::utils::System;
using hyped::utils::concurrent::Thread;


int main (int argc, char* argv[]) {
  System::parseArgs(argc, argv);


  SPI& spi = SPI::getInstance();

  auto log = System::getLogger();

  uint8_t tx[] = {0, 0, 0, 0};
  uint8_t rx[] = {0, 0, 0, 0};
  while (true) {
    spi.transfer(tx, rx, 4);
    log.INFO("DEMO", "received sec, min, hours: %d:%d:%d\n"
      , rx[1], rx[2], rx[3]);
    Thread::sleep(1000);
  }
}
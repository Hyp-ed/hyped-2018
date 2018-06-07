

#include "utils/io/spi.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"
#include "utils/timer.hpp"
#include "utils/io/gpio.hpp"

#include <ctime>
#include <ratio>
#include <chrono>

using hyped::utils::io::SPI;
using hyped::utils::System;
using hyped::utils::Timer;
using hyped::utils::concurrent::Thread;
using hyped::utils::concurrent::BusyThread;
using hyped::utils::io::GPIO;
using hyped::utils::io::gpio::Direction;

using namespace std::chrono;

#define BUSY_NUM  0
#define COUNTS    100000
#define BYTES     7
#define SPI_WRITE_MASK 0x80
Direction kDirection = hyped::utils::io::gpio::kOut;

int main (int argc, char* argv[]) {
  System::parseArgs(argc, argv);

  Thread* busy[BUSY_NUM];
  for (int i=0; i < BUSY_NUM; i++) {
    busy[i] = new BusyThread();
    busy[i]->start();
  }

  SPI& spi = SPI::getInstance();

  auto log = System::getLogger();

  GPIO gpio = GPIO(66, kDirection, log);
  gpio.set();

  uint8_t tx[BYTES] = {};
  uint8_t rx[BYTES] = {};
  // reset clock to time 0
  gpio.clear();
  spi.write(0 | SPI_WRITE_MASK, tx, BYTES);
  gpio.set();
  Timer t;
  t.start();
  uint32_t count = 0;
  while (count < COUNTS) {
    // spi.transfer(tx, rx, BYTES);
    gpio.clear();
    spi.read(0, rx+1, BYTES-1);
    gpio.set();
    count++;
    // log.INFO("DEMO", "received sec, min, hours: %d:%d:%d\n"
    // , rx[1], rx[2], rx[3]);
    // Thread::sleep(1000);
  }
  t.stop();
  uint32_t millis = t.getMillis();
  log.INFO("DEMO", "transfers %d, bytes each %d, busy threads %d\n"
    , COUNTS, BYTES, BUSY_NUM);
  log.INFO("DEMO", "tranfered %d bytes in %d ms\n"
    , COUNTS*BYTES, millis);
  log.INFO("DEMO", "throughput %f B/s\n"
    ,(COUNTS*BYTES*1.0)/(millis/1000.0));
  log.INFO("DEMO", "transfers  %f trans/s\n"
    , (COUNTS*1.0)/(millis/1000));
  log.INFO("DEMO", "average transfer takes %f microsec\n"
    , (millis*1000.0)/COUNTS);

  for (int i=0; i < BUSY_NUM; i++) {
    delete busy[i];
  }
}

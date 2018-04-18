

#include "utils/io/spi.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"

#include <ctime>
#include <ratio>
#include <chrono>

using hyped::utils::io::SPI;
using hyped::utils::System;
using hyped::utils::concurrent::Thread;
using hyped::utils::concurrent::BusyThread;
using namespace std::chrono;

#define BUSY_NUM  5
#define COUNTS    100000
#define BYTES     7

int main (int argc, char* argv[]) {
  System::parseArgs(argc, argv);

  Thread* busy[BUSY_NUM];
  for (int i=0; i < BUSY_NUM; i++) {
    busy[i] = new BusyThread();
    busy[i]->start();
  }

  SPI& spi = SPI::getInstance();

  auto log = System::getLogger();

  uint8_t tx[BYTES] = {};
  uint8_t rx[BYTES] = {};
  log.INFO("DEMO","starting\n");
  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  uint32_t count = 0;
  while (count < COUNTS) {
    spi.transfer(tx, rx, BYTES);
    count++;
    // log.INFO("DEMO", "received sec, min, hours: %d:%d:%d\n"
    //   , rx[1], rx[2], rx[3]);
    // Thread::sleep(1000);
  }
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration<double, std::milli> time_span = t2 - t1;
  uint32_t millis = time_span.count();
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
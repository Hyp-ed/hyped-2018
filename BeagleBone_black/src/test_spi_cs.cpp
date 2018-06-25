

#include "utils/concurrent/thread.hpp"
#include "utils/io/gpio.hpp"
#include "utils/io/spi.hpp"
#include "utils/system.hpp"
#include "utils/logger.hpp"


using hyped::utils::concurrent::Thread;
using hyped::utils::io::GPIO;
using hyped::utils::io::SPI;

class Wait: public Thread {
 public:
  void run() override {
    GPIO pin(66, hyped::utils::io::gpio::kIn);

    while (1) {
      int8_t value = pin.wait();
      printf("value of pin changed to %d\n", value);
    }
  }
};

int main(int argc, char *argv[]) {
  hyped::utils::System::parseArgs(argc, argv);
  hyped::utils::Logger& log = hyped::utils::System::getLogger();

  log.INFO("TEST", "creating gpio wait thread");
  Wait temp;
  temp.start();
  log.INFO("TEST", "gpio wait thread started");
  // temp.join();
  SPI& spi = SPI::getInstance();

  while (1) {
    Thread::sleep(1000);


    log.INFO("TEST", "issueing spi write request");
    uint8_t tx[5] = {0xde, 0xad, 0xbe, 0xef, 0xff};
    uint8_t rx[5] = {};
    printf("0x");
    for (uint8_t val: rx) {
      printf("%x", val);
    }
    printf("\n");
    spi.transfer(tx, rx, 5);
    printf("0x");
    for (uint8_t val: rx) {
      printf("%x", val);
    }
    printf("\n");
    log.INFO("TEST", "spi request finished, received 0x%p"
      , reinterpret_cast<uint64_t*>(rx));
  }
  temp.join();
}


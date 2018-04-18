/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 18. April 2018
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


#include "utils/io/spi.hpp"

#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#ifndef WIN
#include <linux/spi/spidev.h>
#else
#define SPI_IOC_MAGIC             'k'
#define SPI_IOC_WR_MODE           _IOW(SPI_IOC_MAGIC, 1, uint8_t)
#define SPI_IOC_WR_MAX_SPEED_HZ   _IOR(SPI_IOC_MAGIC, 4, uint32_t)
#define SPI_IOC_WR_LSB_FIRST      _IOW(SPI_IOC_MAGIC, 2, uint8_t)
#define SPI_IOC_WR_BITS_PER_WORD  _IOW(SPI_IOC_MAGIC, 3, uint8_t)
struct spi_ioc_transfer {
  uint64_t tx_buf;
  uint64_t rx_buf;

  uint32_t len;
  uint32_t speed_hz;

  uint16_t delay_usecs;
  uint8_t  bits_per_word;
  uint8_t  cs_change;
  uint8_t  tx_nbits;
  uint8_t  rx_nbits;
  uint16_t pad;
};
#define SPI_MSGSIZE(N) \
  ((((N)*(sizeof(struct spi_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
    ? ((N)*(sizeof(struct spi_ioc_transfer))) : 0)
#define SPI_IOC_MESSAGE(N)  _IOW(SPI_IOC_MAGIC, 0, char[SPI_MSGSIZE(N)])
#define SPI_CS_HIGH         0x04
#endif  // ifndef WIN

#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"

// configure SPI
#define SPI_CLK   1000000   // 1MHz
#define SPI_MODE  3
#define SPI_BITS  8         // each word is 1B
#define SPI_MSBFIRST 0
#define SPI_LSBFIRST 1

// DO NOT TOUCH
#define SPI_MAX_SIZE 4096

namespace hyped {
namespace utils {
namespace io {

SPI& SPI::getInstance()
{
  static SPI spi(System::getLogger());
  return spi;
}

SPI::SPI(Logger& log)
    : log_(log)
{
  const char device[] = "/dev/spidev1.0";
  spi_fd_ = open(device, O_RDWR, 0);

  ASSERT(spi_fd_ >= 0);

  // set clock frequency
  uint32_t clock = SPI_CLK;
  if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &clock) < 0) {
    log_.ERR("SPI", "could not set clock frequency\n");
  }

  // set bits per word
  uint8_t bits = SPI_BITS;
  if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
    log_.ERR("SPI", "could not set bits per word\n");
  }

  // set clock mode and CS active low
  uint8_t mode = (SPI_MODE & 0x3) & ~SPI_CS_HIGH;
  if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0) {
    log_.ERR("SPI", "could not set mode\n");
  }

  // set bit order
  uint8_t order = SPI_MSBFIRST;
  if (ioctl(spi_fd_, SPI_IOC_WR_LSB_FIRST, &order) < 0) {
    log_.ERR("SPI", "could not set bit order\n");
  }
}

void SPI::transfer(uint8_t* tx, uint8_t* rx, uint16_t len)
{
  spi_ioc_transfer message = {};

  message.tx_buf = reinterpret_cast<uint64_t>(tx);    // NOLINT
  message.rx_buf = reinterpret_cast<uint64_t>(rx);    // NOLINT
  message.len    = len;

  if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &message) < 0) {
    log_.ERR("SPI", "could not submit TRANSFER message\n");
  }
}

SPI::~SPI()
{
  close(spi_fd_);
}
}}}   // namespace hyped::utils::io

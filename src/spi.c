#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "spi.h"

void spi_init(spi_t* self, const char* dev_path) {
  // open the SPI device
  self->fd = open(dev_path, O_RDWR);
  if (self->fd < 0) {
    fprintf(stderr, "couldn't open SPI device %s\n", dev_path);
    exit(1);
  }

  // SPI config
  uint8_t buf8 = 0;  // CPOL=0, CPHA=0
  IOCTL_WITH_ERR_HANDLING(self->fd, SPI_IOC_WR_MODE, &buf8,
                          "couldn't set SPI mode\n");
  buf8 = 0;  // MSB first
  IOCTL_WITH_ERR_HANDLING(self->fd, SPI_IOC_WR_LSB_FIRST, &buf8,
                          "couldn't set SPI MSB/LSB first\n");
  buf8 = 8;
  IOCTL_WITH_ERR_HANDLING(self->fd, SPI_IOC_WR_BITS_PER_WORD, &buf8,
                          "couldn't set SPI bit count per word\n");
  uint32_t buf32 = 100000;
  IOCTL_WITH_ERR_HANDLING(self->fd, SPI_IOC_WR_MAX_SPEED_HZ, &buf32,
                          "couldn't set SPI max speed\n");
}

void spi_deinit(spi_t* self) {
  // close the SPI device
  close(self->fd);
}

void spi_write_reg8(spi_t* self, uint8_t addr, uint8_t val) {
  struct spi_ioc_transfer xfer[1];
  memset(xfer, 0, sizeof(xfer));
  uint8_t buf[2] = {addr, val};
  buf[0] |= 0x80;  // set the address MSB to indicate write, as opposed to read
  xfer[0].tx_buf = (uint64_t)buf;
  xfer[0].rx_buf = 0;
  xfer[0].len = 2;

  IOCTL_WITH_ERR_HANDLING(self->fd, SPI_IOC_MESSAGE(1), xfer,
                          "couldn't perform SPI register write");
}

void spi_write_bulk(spi_t* self, uint8_t addr, uint8_t* data, uint8_t len) {
  struct spi_ioc_transfer xfer[1];
  memset(xfer, 0, sizeof(xfer));
  uint8_t* buffer = malloc((size_t)(len + 1));
  buffer[0] = addr | 0x80;
  memcpy(buffer + 1, data, (size_t)len);
  xfer[0].tx_buf = (uint64_t)buffer;
  xfer[0].rx_buf = 0;
  xfer[0].len = len + 1;

  IOCTL_WITH_ERR_HANDLING(self->fd, SPI_IOC_MESSAGE(1), xfer,
                          "couldn't perform SPI register write");

  free(buffer);
}

uint8_t spi_read_reg8(spi_t* self, uint8_t addr) {
  struct spi_ioc_transfer xfer[2];
  memset(xfer, 0, sizeof(xfer));
  uint8_t val;
  // don't set the address MSB to indicate read, as opposed to write
  xfer[0].tx_buf = (uint64_t)(&addr);
  xfer[0].rx_buf = 0;
  xfer[0].len = 1;
  xfer[1].tx_buf = 0;
  xfer[1].rx_buf = (uint64_t)(&val);
  xfer[1].len = 1;

  IOCTL_WITH_ERR_HANDLING(self->fd, SPI_IOC_MESSAGE(2), xfer,
                          "couldn't perform SPI register read");

  return val;
}

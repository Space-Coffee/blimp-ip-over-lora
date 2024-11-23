#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "spi.h"

int main(int argc, char** argv) {
  char spi_dev_path[] = "/dev/spidev0.0";

  spi_t spi;
  spi_init(&spi, spi_dev_path);

  printf("configured SPI succesfully\n");

  // some example SPI operations
  uint8_t reg_val = spi_read_reg8(&spi, 0b10110001);
  printf("Received value: %hhX\n", reg_val);

  spi_deinit(&spi);
}

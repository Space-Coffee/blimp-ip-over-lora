#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "gpio.h"
#include "spi.h"

int main(int argc, char** argv) {
  char spi_dev_path[] = "/dev/spidev0.0";

  spi_t spi;
  spi_init(&spi, spi_dev_path);

  printf("configured SPI succesfully\n");

  // some example SPI operations
  uint8_t reg_val = spi_read_reg8(&spi, 0b10110001);
  printf("Received value: %hhX\n", reg_val);

  // GPIO test
  char gpio_dev_path[] = "/dev/gpiochip0";
  gpio_ctl_t gpio_ctl;
  uint32_t gpio_out_lines[] = {25};
  gpio_init(&gpio_ctl, gpio_dev_path, 1, gpio_out_lines, 0, NULL);

  for (uint8_t i = 0; i < 10; i++) {
    gpio_set(&gpio_ctl, i % 2, 0b1);

    struct timespec delay_time;
    delay_time.tv_sec = 0;
    delay_time.tv_nsec = 100000000;
    nanosleep(&delay_time, NULL);
  }

  gpio_deinit(&gpio_ctl);

  spi_deinit(&spi);
}

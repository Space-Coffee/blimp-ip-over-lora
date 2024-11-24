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

  // GPIO test
  char gpio_dev_path[] = "/dev/gpiochip0";
  int gpio_fd = open(gpio_dev_path, 0);
  /*struct gpio_v2_line_request gpio_req;
  memset(&gpio_req, 0, sizeof(gpio_req));
  gpio_req.config.flags=GPIO_V2_LINE_FLAG_OUTPUT;
  gpio_req.config.num_attrs=0;*/
  struct gpiochip_info gpio_cinfo;
  IOCTL_WITH_ERR_HANDLING(gpio_fd, GPIO_GET_CHIPINFO_IOCTL, &gpio_cinfo,
                          "couldn't open GPIO device %s", gpio_dev_path);
  printf("GPIO %s\n  name: %s\n  label: %s\n  lines: %d\n", gpio_dev_path,
         gpio_cinfo.name, gpio_cinfo.label, gpio_cinfo.lines);
  for (uint32_t i = 0; i < gpio_cinfo.lines; i++) {
    struct gpio_v2_line_info gpio_linfo;
    memset(&gpio_linfo, 0, sizeof(gpio_linfo));
    gpio_linfo.offset = i;
    IOCTL_WITH_ERR_HANDLING(gpio_fd, GPIO_V2_GET_LINEINFO_IOCTL, &gpio_linfo,
                            "couldn't get GPIO device %s line %d info",
                            gpio_dev_path, i);
    printf("  line %d:\n    name: %s\n    consumer: %s\n    num_attrs: %d\n", i,
           gpio_linfo.name, gpio_linfo.consumer, gpio_linfo.num_attrs);
  }

  spi_deinit(&spi);
}

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
                            "couldn't get GPIO device %s line %d info\n",
                            gpio_dev_path, i);
    printf("  line %d:\n    name: %s\n    consumer: %s\n    num_attrs: %d\n", i,
           gpio_linfo.name, gpio_linfo.consumer, gpio_linfo.num_attrs);
  }

  struct gpio_v2_line_request gpio_req;
  memset(&gpio_req, 0, sizeof(gpio_req));
  gpio_req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
  gpio_req.config.num_attrs = 0;
  gpio_req.offsets[0] = 25;
  char gpio_consumer[] = "blimp-ip-over-lora";
  strncpy(gpio_req.consumer, gpio_consumer, GPIO_MAX_NAME_SIZE);
  gpio_req.num_lines = 1;
  IOCTL_WITH_ERR_HANDLING(gpio_fd, GPIO_V2_GET_LINE_IOCTL, &gpio_req,
                          "couldn't get GPIO device %s line %d\n",
                          gpio_dev_path, 25);
  for (uint8_t i = 0; i < 10; i++) {
    struct gpio_v2_line_values gpio_vals;
    /*gpio_vals.bits = ((uint64_t)(i % 2)) << 25;*/
    /*gpio_vals.mask = ((uint64_t)1) << 25;*/
    gpio_vals.bits = (uint64_t)(i % 2);
    gpio_vals.mask = 0b1;
    IOCTL_WITH_ERR_HANDLING(gpio_req.fd, GPIO_V2_LINE_SET_VALUES_IOCTL,
                            &gpio_vals, "couldn't set GPIO values\n");

    struct timespec delay_time;
    delay_time.tv_sec = 0;
    delay_time.tv_nsec = 100000000;
    nanosleep(&delay_time, NULL);
  }

  close(gpio_req.fd);
  close(gpio_fd);

  spi_deinit(&spi);
}

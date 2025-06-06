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
#include "sx1278.h"
#include "tun.h"

int main(int argc, char** argv) {
  char spi_dev_path[] = "/dev/spidev0.0";

  spi_t spi;
  spi_init(&spi, spi_dev_path);

  printf("configured SPI succesfully\n");

  // some example SPI operations
  uint8_t reg_val = spi_read_reg8(&spi, 0b10110001);
  printf("received value: %hhX\n", reg_val);

  // GPIO test
  char gpio_dev_path[] = "/dev/gpiochip0";
  gpio_ctl_t gpio_ctl;
  uint32_t gpio_out_lines[] = {25};
  gpio_init(&gpio_ctl, gpio_dev_path, 1, gpio_out_lines, 0, NULL);

  /*for (uint8_t i = 0; i < 10; i++) {
    gpio_set(&gpio_ctl, i % 2, 0b1);

    struct timespec delay_time;
    delay_time.tv_sec = 0;
    delay_time.tv_nsec = 100000000;
    nanosleep(&delay_time, NULL);
  }*/

  sx1278_config_t lora_config = {.rf_freq = 433031200,
                                 .pa_max_power_dbm_x10 = 108,
                                 .pa_out_power_dbm_x10 = 40,
                                 .ocp_max_current_ma = 90,
                                 .lna_gain = 1,
                                 .lna_boost_hf = false};
  sx1278_t lora;
  sx1278_init(&lora, &spi, &gpio_ctl, lora_config);

  printf("SX1278 version: 0x%X\n", sx1278_get_version(&lora));

  // for (uint8_t i = 0; i < 4; i++) {
  //   // uint8_t msg_buf[] = "abcdef";
  //   uint8_t msg_buf[] = "TestXZ_";
  //   msg_buf[6] = '0' + i;
  //   if (sx1278_send(&lora, msg_buf, 7)) {
  //     printf("successfully sent packet!\n");
  //   } else {
  //     fprintf(stderr, "timeout reached when sending packet\n");
  //   }
  //
  //   struct timespec delay_time;
  //   // delay_time.tv_sec = 0;
  //   // delay_time.tv_nsec = 500000000;  // 500ms
  //   delay_time.tv_sec = 2;
  //   delay_time.tv_nsec = 0;  // 500ms
  //   nanosleep(&delay_time, NULL);
  // }

  sx1278_receive(&lora);

  for (uint16_t i = 0; i < 1000; i++) {
    uint8_t* msg_buf_ptr;
    uint8_t packet_len = sx1278_get_received(&lora, &msg_buf_ptr);
    if (packet_len) {
      printf("Received message: \"");
      for (uint8_t j = 0; j < packet_len; j++) {
        printf("%c", msg_buf_ptr[j]);
      }
      printf("\" = {");
      for (uint8_t j = 0; j < packet_len; j++) {
        printf("%hhX ", msg_buf_ptr[j]);
      }
      printf("}\n");
      free(msg_buf_ptr);
    }

    struct timespec delay_time;
    delay_time.tv_sec = 0;
    delay_time.tv_nsec = 20000000;  // 20ms
    nanosleep(&delay_time, NULL);
  }

  int tun_fd = tun_alloc("ip-over-lora");
  printf("configured tun\n");

  uint8_t tun_buf[2048];
  while (1) {
    int count = read(tun_fd, tun_buf, sizeof(tun_buf));
    if (count > 0) {
      printf("received %d bytes from tun\n", count);
    }
  }

  gpio_deinit(&gpio_ctl);

  spi_deinit(&spi);
}

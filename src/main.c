#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define IOCTL_WITH_ERR_HANDLING(fd, ctl, val, err_msg, ...) \
  do {                                                      \
    if (ioctl(fd, ctl, val) < 0) {                          \
      fprintf(stderr, err_msg __VA_OPT__(, ) __VA_ARGS__);  \
      fprintf(stderr, "errno = %d\n", errno);               \
      exit(1);                                              \
    }                                                       \
  } while (0);

int main(int argc, char** argv) {
  char spi_dev_path[] = "/dev/spidev10.0";

  // open the SPI device
  int spi_fd = open(spi_dev_path, O_RDWR);
  if (spi_fd < 0) {
    fprintf(stderr, "couldn't open SPI device %s\n", spi_dev_path);
    exit(1);
  }

  // SPI config
  uint8_t buf8 = SPI_CPHA | SPI_CPOL;
  IOCTL_WITH_ERR_HANDLING(spi_fd, SPI_IOC_WR_MODE, &buf8,
                          "couldn't set SPI mode\n");
  buf8 = 0;  // MSB first
  IOCTL_WITH_ERR_HANDLING(spi_fd, SPI_IOC_WR_LSB_FIRST, &buf8,
                          "couldn't set SPI MSB/LSB first\n");
  buf8 = 8;
  IOCTL_WITH_ERR_HANDLING(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &buf8,
                          "couldn't set SPI bit count per word\n");
  uint32_t buf32 = 1000000;
  IOCTL_WITH_ERR_HANDLING(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &buf32,
                          "couldn't set SPI max speed\n");

  printf("configured SPI succesfully\n");

  // close the SPI device
  close(spi_fd);
}

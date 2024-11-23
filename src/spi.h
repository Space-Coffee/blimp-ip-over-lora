#ifndef _BLIMP_SPI_H
#define _BLIMP_SPI_H

#include <stdint.h>

#define IOCTL_WITH_ERR_HANDLING(fd, ctl, val, err_msg, ...) \
  do {                                                      \
    if (ioctl(fd, ctl, val) < 0) {                          \
      fprintf(stderr, err_msg __VA_OPT__(, ) __VA_ARGS__);  \
      fprintf(stderr, "errno = %d\n", errno);               \
      exit(1);                                              \
    }                                                       \
  } while (0);

typedef struct {
  int fd;
} spi_t;

extern void spi_init(spi_t* self, const char* dev_path);
extern void spi_deinit(spi_t* self);
extern void spi_write_reg8(spi_t* self, uint8_t addr, uint8_t val);
extern uint8_t spi_read_reg8(spi_t* self, uint8_t addr);

#endif

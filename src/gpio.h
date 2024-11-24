#ifndef _BLIMP_GPIO_H
#define _BLIMP_GPIO_H

#include <stdint.h>

typedef struct {
  int fd;
  uint8_t out_lines_count;
  uint32_t out_lines[64];
  int out_lines_fd;
  uint8_t in_lines_count;
  uint32_t in_lines[64];
  int in_lines_fd;
} gpio_ctl_t;

extern void gpio_init(gpio_ctl_t* self,
                      char* dev_path,
                      uint8_t out_lines_count,
                      uint32_t* out_lines,
                      uint8_t in_lines_count,
                      uint32_t* in_lines);
extern void gpio_deinit(gpio_ctl_t* self);
extern void gpio_set(gpio_ctl_t* self, uint64_t val, uint64_t mask);
extern uint64_t gpio_get(gpio_ctl_t* self, uint64_t mask);

#endif

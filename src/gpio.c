#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "gpio.h"
#include "spi.h"

static char gpio_consumer_name[] = "blimp-ip-over-lora";

void gpio_init(gpio_ctl_t* self,
               char* dev_path,
               uint8_t out_lines_count,
               uint32_t* out_lines,
               uint8_t in_lines_count,
               uint32_t* in_lines) {
  self->fd = open(dev_path, 0);
  if (self->fd < 0) {
    fprintf(stderr, "couldn't open GPIO device %s\nerrno = %d", dev_path,
            errno);
    exit(1);
  }

  if (out_lines_count > 0) {
    struct gpio_v2_line_request out_req;
    memset(&out_req, 0, sizeof(out_req));
    out_req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    out_req.config.num_attrs = 0;  // do we need attributes?
    memcpy(out_req.offsets, out_lines, out_lines_count);
    out_req.num_lines = out_lines_count;
    strncpy(out_req.consumer, gpio_consumer_name, GPIO_MAX_NAME_SIZE);

    IOCTL_WITH_ERR_HANDLING(self->fd, GPIO_V2_GET_LINE_IOCTL, &out_req,
                            "couldn't open GPIO device %s output lines\n",
                            dev_path);
    self->out_lines_fd = out_req.fd;
    if (self->out_lines_fd < 0) {
      fprintf(stderr, "couldn't open GPIO device %s output lines\nerrno = %d",
              dev_path, errno);
      exit(1);
    }
  } else {
    self->out_lines_fd = -1;
  }

  if (in_lines_count > 0) {
    struct gpio_v2_line_request in_req;
    memset(&in_req, 0, sizeof(in_req));
    in_req.config.flags = GPIO_V2_LINE_FLAG_INPUT;
    in_req.config.num_attrs = 0;  // do we need attributes?
    memcpy(in_req.offsets, in_lines, in_lines_count);
    in_req.num_lines = in_lines_count;
    strncpy(in_req.consumer, gpio_consumer_name, GPIO_MAX_NAME_SIZE);

    IOCTL_WITH_ERR_HANDLING(self->fd, GPIO_V2_GET_LINE_IOCTL, &in_req,
                            "couldn't open GPIO device %s input lines\n",
                            dev_path);
    self->in_lines_fd = in_req.fd;
    if (self->in_lines_fd < 0) {
      fprintf(stderr, "couldn't open GPIO device %s in lines\nerrno = %d",
              dev_path, errno);
      exit(1);
    }
  } else {
    self->in_lines_fd = -1;
  }
}

void gpio_deinit(gpio_ctl_t* self) {
  if (self->out_lines_fd >= 0) {
    close(self->out_lines_fd);
  }
  if (self->in_lines_fd >= 0) {
    close(self->in_lines_fd);
  }
  close(self->fd);
}

void gpio_set(gpio_ctl_t* self, uint64_t val, uint64_t mask) {
  struct gpio_v2_line_values vals;
  vals.bits = val;
  vals.mask = mask;
  IOCTL_WITH_ERR_HANDLING(self->out_lines_fd, GPIO_V2_LINE_SET_VALUES_IOCTL,
                          &vals, "couldn't set GPIO values\n");
}

uint64_t gpio_get(gpio_ctl_t* self, uint64_t mask) {
  struct gpio_v2_line_values vals;
  vals.mask = mask;
  IOCTL_WITH_ERR_HANDLING(self->in_lines_fd, GPIO_V2_LINE_GET_VALUES_IOCTL,
                          &vals, "couldn't get GPIO values\n");
  return vals.bits;
}

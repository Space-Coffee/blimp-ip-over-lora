#ifndef _BLIMP_NRF905_H
#define _BLIMP_NRF905_H

#include <stdint.h>

#include "gpio.h"
#include "spi.h"

typedef struct {
  spi_t* spi;
  gpio_ctl_t* gpio;
} nrf905_t;

extern void nrf905_init(nrf905_t* self, spi_t* spi, gpio_ctl_t* gpio);
extern void nrf905_deinit(nrf905_t* self);

#endif

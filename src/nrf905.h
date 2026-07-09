#ifndef _BLIMP_NRF905_H
#define _BLIMP_NRF905_H

#include <stdint.h>

#include "gpio.h"
#include "spi.h"

typedef struct {
  uint32_t tx_en_pin;
  uint32_t trw_ce_pin;
  uint32_t dr_pin;  // Data ready
} nrf905_pins_t;

typedef struct {
  spi_t* spi;
  gpio_ctl_t* gpio;
  nrf905_pins_t pins;
} nrf905_t;

extern void nrf905_init(nrf905_t* self,
                        spi_t* spi,
                        gpio_ctl_t* gpio,
                        nrf905_pins_t pins);
extern void nrf905_deinit(nrf905_t* self);
extern void nrf905_transmit(nrf905_t* self,
                            uint32_t dest_addr,
                            uint8_t* payload);

#endif

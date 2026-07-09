#include <stdint.h>
#include <string.h>

#include "gpio.h"
#include "nrf905.h"
#include "spi.h"

void nrf905_spi_read_configs(nrf905_t* self,
                             uint8_t reg_start,
                             uint8_t* values,
                             uint8_t len) {
  uint8_t msg_tx_buf[65];
  uint8_t msg_rx_buf[65];
  msg_tx_buf[0] = 0x10 | reg_start;
  memset(msg_tx_buf + 1, 0, len);
  spi_transact(self->spi, len + 1, msg_tx_buf, msg_rx_buf);
}

void nrf905_spi_write_configs(nrf905_t* self,
                              uint8_t reg_start,
                              uint8_t* values,
                              uint8_t len) {
  uint8_t msg_tx_buf[65];
  msg_tx_buf[0] = 0x00 | reg_start;
  memcpy(msg_tx_buf + 1, values, len);
  spi_transact(self->spi, len + 1, msg_tx_buf, NULL);
}

void nrf905_init(nrf905_t* self,
                 spi_t* spi,
                 gpio_ctl_t* gpio,
                 nrf905_pins_t pins) {
  self->spi = spi;
  self->gpio = gpio;
  self->pins = pins;

  uint16_t channel_num = 108;
  uint8_t auto_retran = 0;
  uint8_t rx_red_power = 0;
  uint8_t pa_power = 0;
  uint8_t hfreq_pll = 0;
  uint8_t rx_addr_width = 0b100;
  uint8_t tx_addr_width = rx_addr_width;
  uint8_t rx_payload_width = 0b100000;
  uint8_t tx_payload_width = rx_payload_width;
  uint32_t rx_addr = 0xE7E7E7E7;
  uint8_t crc_mode = 1;
  uint8_t crc_en = 1;
  uint8_t cryst_freq = 0b100;
  uint8_t up_clk_en = 1;
  uint8_t up_clk_freq = 0b11;

  uint8_t configs[10];
  memset(configs, 0, sizeof(configs));
  configs[0] = (uint8_t)(channel_num);
  configs[1] = (auto_retran << 5) | (rx_red_power << 4) | (pa_power << 2) |
               (hfreq_pll << 1) | ((uint8_t)(channel_num >> 8));
  configs[2] = (tx_addr_width << 4) | rx_addr_width;
  configs[3] = rx_payload_width;
  configs[4] = tx_payload_width;
  configs[5] = (uint8_t)rx_addr;
  configs[6] = (uint8_t)(rx_addr >> 8);
  configs[7] = (uint8_t)(rx_addr >> 16);
  configs[8] = (uint8_t)(rx_addr >> 24);
  configs[9] = (crc_mode << 7) | (crc_en << 6) | (cryst_freq << 3) |
               (up_clk_en << 2) | up_clk_freq;
  nrf905_spi_write_configs(self, 0, configs, 10);
}

void nrf905_deinit(nrf905_t* self) {}

void nrf905_transmit(nrf905_t* self, uint32_t dest_addr, uint8_t* payload) {
  uint8_t msg_buf[33];

  // Set TX address
  msg_buf[0] = 0b00100010;
  msg_buf[1] = (uint8_t)dest_addr;
  msg_buf[2] = (uint8_t)(dest_addr >> 8);
  msg_buf[3] = (uint8_t)(dest_addr >> 16);
  msg_buf[4] = (uint8_t)(dest_addr >> 24);
  spi_transact(self->spi, 5, msg_buf, NULL);

  // Send payload
  msg_buf[0] = 0b00100000;
  memcpy(msg_buf + 1, payload, 32);

  // TODO: Set GPIOs
}

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "gpio.h"
#include "spi.h"
#include "sx1278.h"

void sx1278_init(sx1278_t* self,
                 spi_t* spi,
                 gpio_ctl_t* gpio,
                 uint64_t rf_freq) {
  self->spi = spi;
  self->gpio = gpio;

  sx1278_reset(self);

  self->dev_detected = sx1278_check_device(self);
  if (!self->dev_detected) {
    fprintf(stderr, "LoRa SX1278 not detected!\n");
    exit(1);
  }

  // mode
  sx1278_set_mode(self, (1 << 7 /*LoRa mode*/) | (0b001 << 0 /*standby*/));

  // frequency
  uint64_t rf_freq_reg =
      (rf_freq << 19) /
      32000000;  // TODO: verify if 32MHz is correct oscillator frequency
  spi_write_reg8(self->spi, SX1278_REG_FR_MSB, (uint8_t)(rf_freq_reg >> 16));
  spi_write_reg8(self->spi, SX1278_REG_FR_MID, (uint8_t)(rf_freq_reg >> 8));
  spi_write_reg8(self->spi, SX1278_REG_FR_LSB, (uint8_t)rf_freq_reg);

  // PA config
  spi_write_reg8(self->spi, SX1278_REG_PA_CONFIG,
                 (0 << 7 /*PA output = RFO*/) | (0 << 4 /*Pmax = 10.8dBm*/) |
                     (8 << 0 /*Pout = 3.8dBm*/));
  // OCP - overload current protection
  spi_write_reg8(
      self->spi, SX1278_REG_OCP,
      (1 << 5 /*OCP on*/) | (0 << 0 /*45mA*/));  // TODO: adjust max current
  // LNA
  spi_write_reg8(self->spi, SX1278_REG_LNA, (0b110 << 5 /*G6 - minimum gain*/));

  // setup FIFO
  spi_write_reg8(self->spi, SX1278_REG_FIFO_TX_BASE_ADDR, 0);
  spi_write_reg8(self->spi, SX1278_REG_FIFO_RX_BASE_ADDR, 0);
  // LoRa modem config
  spi_write_reg8(self->spi, SX1278_REG_MODEM_CONFIG_1,
                 (0b011 << 4 /*20.8kHz bandwidth*/) |
                     (0b010 << 1 /*4/6 coding rate*/) |
                     (0 << 0 /*explicit header*/));
  spi_write_reg8(self->spi, SX1278_REG_MODEM_CONFIG_2,
                 (7 << 4 /*spreading factor 128 chips/symbol*/) |
                     (1 << 2 /*CRC on*/) | (0 << 0 /*RX timeout MSB*/));
  spi_write_reg8(self->spi, SX1278_REG_SYMB_TIMEOUT_LSB,
                 0x64 /*RX timeout LSB*/);
  spi_write_reg8(self->spi, SX1278_REG_MODEM_CONFIG_3,
                 (0 << 3 /*disable low data rate optimization*/) |
                     (0 << 2 /*disable AGC*/));
  // preamble length
  spi_write_reg8(self->spi, SX1278_REG_PREAMBLE_MSB, 0);
  spi_write_reg8(self->spi, SX1278_REG_PREAMBLE_LSB, 8);
  // hop period
  spi_write_reg8(self->spi, SX1278_REG_HOP_PERIOD,
                 0 /*disable frequency hopping*/);

  // sync word
  spi_write_reg8(self->spi, SX1278_REG_SYNC_WORD, 0x12);
}

void sx1278_deinit(sx1278_t* self) {}

void sx1278_reset(sx1278_t* self) {
  struct timespec delay_time;
  delay_time.tv_sec = 0;
  delay_time.tv_nsec = 20000000;  // 20ms
  nanosleep(&delay_time, NULL);

  gpio_set(self->gpio, 0 << 0, 1 << 0);

  delay_time.tv_nsec = 50000000;  // 50ms
  nanosleep(&delay_time, NULL);

  gpio_set(self->gpio, 1 << 0, 1 << 0);

  delay_time.tv_nsec = 20000000;  // 20ms
  nanosleep(&delay_time, NULL);
}

bool sx1278_check_device(sx1278_t* self) {
  // see:
  // https://github.com/StuartsProjects/SX12XX-LoRa/blob/master/src/SX127XLT.cpp#L298
  uint8_t tmp1, tmp2;
  tmp1 = spi_read_reg8(self->spi, SX1278_REG_FR_MID);
  spi_write_reg8(self->spi, SX1278_REG_FR_MID, tmp1 + 1);
  tmp2 = spi_read_reg8(self->spi, SX1278_REG_FR_MID);
  spi_write_reg8(self->spi, SX1278_REG_FR_MID, tmp1);
  return tmp2 == tmp1 + 1;
}

void sx1278_set_mode(sx1278_t* self, uint8_t mode) {
  spi_write_reg8(self->spi, SX1278_REG_OP_MODE, mode);
}

bool sx1278_send(sx1278_t* self, uint8_t* data, uint8_t len) {
  spi_write_reg8(self->spi, SX1278_REG_FIFO_ADDR_PTR, 0);
  spi_write_reg8(self->spi, SX1278_REG_OP_MODE,
                 (1 << 7 /*LoRa mode*/) | (0b011 << 0 /*TX mode*/));

  spi_write_bulk(self->spi, SX1278_REG_FIFO, data, len);
  spi_write_reg8(self->spi, SX1278_REG_PAYLOAD_LENGTH, len);
  sx1278_set_mode(self, (1 << 7 /*LoRa mode*/) | (0b011 << 0 /*TX*/));

  uint8_t irq_flags;
  uint8_t counter = 255;
  do {
    struct timespec delay_time;
    delay_time.tv_sec = 0;
    delay_time.tv_nsec = 10000000;
    nanosleep(&delay_time, NULL);

    irq_flags = spi_read_reg8(self->spi, SX1278_REG_IRQ_FLAGS);
    counter--;

  } while (irq_flags & (1 << 3 /*TX done*/) && counter > 0);

  return counter > 0;
}

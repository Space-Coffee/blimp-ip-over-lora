#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "gpio.h"
#include "spi.h"
#include "sx1278.h"

void sx1278_init(sx1278_t* self,
                 spi_t* spi,
                 gpio_ctl_t* gpio,
                 sx1278_config_t config) {
  self->spi = spi;
  self->gpio = gpio;
  self->config = config;

  sx1278_reset(self);

  self->dev_detected = sx1278_check_device(self);
  if (!self->dev_detected) {
    fprintf(stderr, "LoRa SX1278 not detected!\n");
    exit(1);
  }

  // mode
  sx1278_set_mode(self, (1 << 7 /*LoRa mode*/) | (0b000 << 0 /*sleep*/));

  // frequency
  uint64_t rf_freq_reg =
      (config.rf_freq << 19) /
      32000000;  // TODO: verify if 32MHz is correct oscillator frequency
  spi_write_reg8(self->spi, SX1278_REG_FR_MSB, (uint8_t)(rf_freq_reg >> 16));
  spi_write_reg8(self->spi, SX1278_REG_FR_MID, (uint8_t)(rf_freq_reg >> 8));
  spi_write_reg8(self->spi, SX1278_REG_FR_LSB, (uint8_t)rf_freq_reg);

  // PA config
  uint8_t pa_max_power_reg = (config.pa_max_power_dbm_x10 - 108) / 6;
  fprintf(stderr, "pa_max_power_reg = %hhd\n", pa_max_power_reg);
  uint8_t pa_out_power_reg = (config.pa_out_power_dbm_x10 - 20) / 10;
  fprintf(stderr, "pa_out_power_reg = %hhd\n", pa_out_power_reg);
  spi_write_reg8(self->spi, SX1278_REG_PA_CONFIG,
                 (1 << 7 /*PA output = PA_BOOST*/) |
                     (pa_max_power_reg << 4 /*Pmax*/) |
                     (pa_out_power_reg << 0 /*Pout*/));
  // OCP - overload current protection
  uint8_t ocp_trim_reg;
  if (config.ocp_max_current_ma <= 120) {
    ocp_trim_reg = (config.ocp_max_current_ma - 45) / 5;
  } else if (config.ocp_max_current_ma <= 240) {
    ocp_trim_reg = (config.ocp_max_current_ma + 30) / 10;
  } else {
    ocp_trim_reg = 27;
  }
  spi_write_reg8(
      self->spi, SX1278_REG_OCP,
      (1 << 5 /*OCP on*/) |
          (ocp_trim_reg << 0 /*OCP trim*/));  // TODO: adjust max current
  // LNA
  spi_write_reg8(
      self->spi, SX1278_REG_LNA,
      ((config.lna_gain == 0 ? 1 : config.lna_gain) << 5 /*LNA gain*/) |
          ((config.lna_boost_hf ? 0b11 : 0b00) << 0 /*high freq. LNA boost*/));

  // setup FIFO
  spi_write_reg8(self->spi, SX1278_REG_FIFO_TX_BASE_ADDR, 0);
  spi_write_reg8(self->spi, SX1278_REG_FIFO_RX_BASE_ADDR, 0);

  // LoRa modem config
  spi_write_reg8(self->spi, SX1278_REG_MODEM_CONFIG_1,
                 (0b0111 << 4 /*125kHz bandwidth*/) |
                     (0b100 << 1 /*4/8 coding rate*/) |
                     (0 << 0 /*explicit header*/));
  spi_write_reg8(self->spi, SX1278_REG_MODEM_CONFIG_2,
                 (9 << 4 /*spreading factor 9 (512 chips/symbol)*/) |
                     (0 << 3 /*Non-continuous TX mode*/) |
                     (1 << 2 /*CRC enable*/) | (0 << 0 /*RX timeout MSB*/));
  spi_write_reg8(self->spi, SX1278_REG_SYMB_TIMEOUT_LSB,
                 0x64 /*RX timeout LSB*/);
  spi_write_reg8(self->spi, SX1278_REG_MODEM_CONFIG_3,
                 (0 << 3 /*disable low data rate optimization*/) |
                     ((config.lna_gain == 0) << 2 /*AGC*/));

  // detection
  spi_write_reg8(self->spi, SX1278_REG_DETECT_OPTIMIZE,
                 (0x3 << 0 /*SF7-SF12*/));
  spi_write_reg8(self->spi, SX1278_REG_DETECTION_THRESHOLD, 0xA /*SF7-SF12*/);

  // preamble length
  uint16_t preamble_length = 64;
  spi_write_reg8(self->spi, SX1278_REG_PREAMBLE_MSB, preamble_length >> 8);
  spi_write_reg8(self->spi, SX1278_REG_PREAMBLE_LSB, preamble_length & 0xFF);

  // hop period
  spi_write_reg8(self->spi, SX1278_REG_HOP_PERIOD,
                 0 /*disable frequency hopping*/);

  // sync word
  spi_write_reg8(self->spi, SX1278_REG_SYNC_WORD, 0x91);

  sx1278_set_mode(self, (1 << 7 /*LoRa Mode*/) | (0b001 << 0 /*standby*/));
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

uint8_t sx1278_get_version(sx1278_t* self) {
  return spi_read_reg8(self->spi, SX1278_REG_VERSION);
}

void sx1278_set_mode(sx1278_t* self, uint8_t mode) {
  spi_write_reg8(self->spi, SX1278_REG_OP_MODE, mode);
}

bool sx1278_send(sx1278_t* self, uint8_t* data, uint8_t len) {
  spi_write_reg8(self->spi, SX1278_REG_PAYLOAD_LENGTH, len);

  spi_write_reg8(self->spi, SX1278_REG_FIFO_ADDR_PTR, 0);
  spi_write_bulk(self->spi, SX1278_REG_FIFO, data, len);

  // Clear the IRQ flag
  spi_write_reg8(self->spi, SX1278_REG_IRQ_FLAGS, 0xFF);

  struct timespec delay_time;
  delay_time.tv_sec = 0;
  delay_time.tv_nsec = 1000000;  // 1ms
  nanosleep(&delay_time, NULL);

  sx1278_set_mode(self, (1 << 7 /*LoRa mode*/) | (0b011 << 0 /*TX*/));

  uint8_t irq_flags;
  int16_t counter = 1000;
  do {
    delay_time.tv_sec = 0;
    delay_time.tv_nsec = 100000000;  // 100ms
    nanosleep(&delay_time, NULL);

    irq_flags = spi_read_reg8(self->spi, SX1278_REG_IRQ_FLAGS);
    counter--;

  } while ((irq_flags & (1 << 3 /*TX done*/)) == 0 && counter > 0);

  /*delay_time.tv_sec = 4;
  delay_time.tv_nsec = 0;
  nanosleep(&delay_time, NULL);*/

  printf("OP_MODE = 0x%X\n", spi_read_reg8(self->spi, SX1278_REG_OP_MODE));
  printf("IRQ_FLAGS = 0x%X\n", spi_read_reg8(self->spi, SX1278_REG_IRQ_FLAGS));

  // Clear the IRQ flag
  spi_write_reg8(self->spi, SX1278_REG_IRQ_FLAGS, 0xFF);

  return counter > 0;
}

void sx1278_receive(sx1278_t* self) {
  spi_write_reg8(self->spi, SX1278_REG_FIFO_RX_CURRENT_ADDR, 0);
  sx1278_set_mode(self,
                  (1 << 7 /*LoRa mode*/) | (0b101 << 0 /*RX continuous*/));
}

uint8_t sx1278_get_received(sx1278_t* self, uint8_t** result_ptr) {
  uint8_t irq_flags = spi_read_reg8(self->spi, SX1278_REG_IRQ_FLAGS);
  spi_write_reg8(self->spi, SX1278_REG_IRQ_FLAGS,
                 0xFF);  // Clear all the IRQ flags
  if (irq_flags & (1 << 6 /*RX done*/)) {
    if (irq_flags & (1 << 5 /*CRC error*/)) {
      fprintf(stderr, "Received packet with CRC error!\n");
    } else {
      // Get packet length and allocate buffer for it
      uint8_t packet_len = spi_read_reg8(self->spi, SX1278_REG_RX_NB_BYTES);
      *result_ptr = malloc(packet_len);
      // Set FIFO read pointer to start of the packet payload
      spi_write_reg8(self->spi, SX1278_REG_FIFO_ADDR_PTR,
                     spi_read_reg8(self->spi, SX1278_REG_FIFO_RX_CURRENT_ADDR));
      // Read the payload
      spi_read_bulk(self->spi, SX1278_REG_FIFO, packet_len, *result_ptr);
      return packet_len;
    }
  }
  return 0;
}

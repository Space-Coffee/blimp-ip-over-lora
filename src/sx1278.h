#ifndef _BLIMP_SX1278_H
#define _BLIMP_SX1278_H

#include <stdbool.h>
#include <stdint.h>

#include "gpio.h"
#include "spi.h"

#define SX1278_REG_FIFO 0x00
#define SX1278_REG_OP_MODE 0x01
#define SX1278_REG_FR_MSB 0x06
#define SX1278_REG_FR_MID 0x07
#define SX1278_REG_FR_LSB 0x08
#define SX1278_REG_PA_CONFIG 0x09
#define SX1278_REG_PA_RAMP 0x0A
#define SX1278_REG_OCP 0x0B
#define SX1278_REG_LNA 0x0C
#define SX1278_REG_FIFO_ADDR_PTR 0x0D
#define SX1278_REG_FIFO_TX_BASE_ADDR 0x0E
#define SX1278_REG_FIFO_RX_BASE_ADDR 0x0F
#define SX1278_REG_FIFO_RX_CURRENT_ADDR 0x10
#define SX1278_REG_IRQ_FLAGS_MASK 0x11
#define SX1278_REG_IRQ_FLAGS 0x12
#define SX1278_REG_RX_NB_BYTES 0x13
#define SX1278_REG_RX_HEADER_CNT_VALUE_MSB 0x14
#define SX1278_REG_RX_HEADER_CNT_VALUE_LSB 0x15
#define SX1278_REG_RX_PACKET_CNT_VALUE_MSB 0x16
#define SX1278_REG_RX_PACKET_CNT_VALUE_LSB 0x17
#define SX1278_REG_MODEM_STAT 0x18
#define SX1278_REG_PKT_SNR_VALUE 0x19
#define SX1278_REG_PKT_RSSI_VALUE 0x1A
#define SX1278_REG_RSSI_VALUE 0x1B
#define SX1278_REG_HOP_CHANNEL 0x1C
#define SX1278_REG_MODEM_CONFIG_1 0x1D
#define SX1278_REG_MODEM_CONFIG_2 0x1E
#define SX1278_REG_SYMB_TIMEOUT_LSB 0x1F
#define SX1278_REG_PREAMBLE_MSB 0x20
#define SX1278_REG_PREAMBLE_LSB 0x21
#define SX1278_REG_PAYLOAD_LENGTH 0x22
#define SX1278_REG_MAX_PAYLOAD_LENGTH 0x23
#define SX1278_REG_HOP_PERIOD 0x24
#define SX1278_REG_FIFO_RX_BYTE_ADDR 0x25
#define SX1278_REG_MODEM_CONFIG_3 0x26
#define SX1278_REG_FEI_MSB 0x28
#define SX1278_REG_FEI_MID 0x29
#define SX1278_REG_FEI_LSB 0x2A
#define SX1278_REG_RSSI_WIDEBAND 0x2C
#define SX1278_REG_DETECT_OPTIMIZE 0x31
#define SX1278_REG_INVERT_IQ 0x33
#define SX1278_REG_DETECTION_THRESHOLD 0x37
#define SX1278_REG_SYNC_WORD 0x39
#define SX1278_REG_VERSION 0x42

typedef struct {
  spi_t* spi;
  gpio_ctl_t* gpio;
  bool dev_detected;
} sx1278_t;

extern void sx1278_init(sx1278_t* self,
                        spi_t* spi,
                        gpio_ctl_t* gpio,
                        uint64_t rf_freq);
extern void sx1278_deinit(sx1278_t* self);
extern void sx1278_reset(sx1278_t* self);
extern bool sx1278_check_device(sx1278_t* self);
extern uint8_t sx1278_get_version(sx1278_t* self);
extern void sx1278_set_mode(sx1278_t* self, uint8_t mode);
extern bool sx1278_send(sx1278_t* self, uint8_t* data, uint8_t len);

#endif

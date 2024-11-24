#include "sx1278.h"
#include "spi.h"

void sx1278_init(sx1278_t* self, spi_t* spi) {
  self->spi = spi;

  sx1278_reset(self);

  self->dev_detected = sx1278_check_device(self);
}

void sx1278_reset(sx1278_t* self) {
  // TODO: reset pin
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

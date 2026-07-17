#include <fcntl.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <stdint.h>

// Zig has issues with some more complex macros
const uint64_t SPI_IOC_MESSAGE_1 = SPI_IOC_MESSAGE(1);
const uint64_t SPI_IOC_MESSAGE_2 = SPI_IOC_MESSAGE(2);

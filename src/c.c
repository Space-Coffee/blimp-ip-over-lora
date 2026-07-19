#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>

// Zig has issues with some more complex macros
const uint64_t SPI_IOC_MESSAGE_1 = SPI_IOC_MESSAGE(1);
const uint64_t SPI_IOC_MESSAGE_2 = SPI_IOC_MESSAGE(2);

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "spi.h"

int tun_alloc(char* tun_name) {
  // Open special file used to manage tun devices
  int fd = open("/dev/net/tun", O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    return -1;
  }

  // Create tun
  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_ifrn.ifrn_name, tun_name, IFNAMSIZ);
  ifr.ifr_ifru.ifru_flags = IFF_TUN;
  IOCTL_WITH_ERR_HANDLING(fd, TUNSETIFF, &ifr, "couldn't create tun device");
  char assigned_name[IFNAMSIZ];
  strncpy(assigned_name, ifr.ifr_ifrn.ifrn_name, sizeof(assigned_name));

  // Open dummy socket
  int socket_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);

  // Address
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_ifrn.ifrn_name, assigned_name, IFNAMSIZ);
  struct sockaddr_in* addr = (struct sockaddr_in*)&ifr.ifr_ifru.ifru_addr;
  addr->sin_family = AF_INET;
  inet_pton(AF_INET, "10.12.34.1", &addr->sin_addr);
  IOCTL_WITH_ERR_HANDLING(socket_fd, SIOCSIFADDR, &ifr,
                          "couldn't set tun address");
  // Netmask
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_ifrn.ifrn_name, assigned_name, IFNAMSIZ);
  addr = (struct sockaddr_in*)&ifr.ifr_ifru.ifru_netmask;
  addr->sin_family = AF_INET;
  inet_pton(AF_INET, "255.255.255.0", &addr->sin_addr);
  IOCTL_WITH_ERR_HANDLING(socket_fd, SIOCSIFNETMASK, &ifr,
                          "couldn't set tun netmask");
  // Flags
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_ifrn.ifrn_name, assigned_name, IFNAMSIZ);
  IOCTL_WITH_ERR_HANDLING(socket_fd, SIOCGIFFLAGS, &ifr,
                          "couldn't get tun flags");
  ifr.ifr_ifru.ifru_flags |=
      (short)(IFF_UP | IFF_LOWER_UP | IFF_NOARP | IFF_MULTICAST |
              IFF_POINTOPOINT | IFF_RUNNING);
  /*ifr.ifr_ifru.ifru_flags |= (short)(IFF_UP | IFF_RUNNING);*/
  IOCTL_WITH_ERR_HANDLING(socket_fd, SIOCSIFFLAGS, &ifr,
                          "couldn't set tun flags");

  close(socket_fd);

  return fd;
}

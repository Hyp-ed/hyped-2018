/*
 * Authors: M. Kristien
 * Organisation: HYPED
 * Date: 14. March 2018
 * Description:
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include "utils/io/can.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>


#define CANID_DELIM '#'
#define DATA_SEPERATOR '.'

namespace hyped {
namespace utils {
namespace io {

/* CAN DLC to real data length conversion helpers */

static const unsigned char dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7,
          8, 12, 16, 20, 24, 32, 48, 64};

/* get data length from can_dlc with sanitized can_dlc */
unsigned char can_dlc2len(unsigned char can_dlc)
{
  return dlc2len[can_dlc & 0x0F];
}

static const unsigned char len2dlc[] = {0, 1, 2, 3, 4, 5, 6, 7, 8,    /* 0 - 8 */
          9, 9, 9, 9,       /* 9 - 12 */
          10, 10, 10, 10,       /* 13 - 16 */
          11, 11, 11, 11,       /* 17 - 20 */
          12, 12, 12, 12,       /* 21 - 24 */
          13, 13, 13, 13, 13, 13, 13, 13,   /* 25 - 32 */
          14, 14, 14, 14, 14, 14, 14, 14,   /* 33 - 40 */
          14, 14, 14, 14, 14, 14, 14, 14,   /* 41 - 48 */
          15, 15, 15, 15, 15, 15, 15, 15,   /* 49 - 56 */
          15, 15, 15, 15, 15, 15, 15, 15};  /* 57 - 64 */


unsigned char can_len2dlc(unsigned char len)
{
  if (len > 64)
    return 0xF;

  return len2dlc[len];
}

unsigned char asc2nibble(char c)
{
  if ((c >= '0') && (c <= '9'))
    return c - '0';

  if ((c >= 'A') && (c <= 'F'))
    return c - 'A' + 10;

  if ((c >= 'a') && (c <= 'f'))
    return c - 'a' + 10;

  return 16; /* error */
}

int parse_canframe(char *cs, struct canfd_frame *cf)
{
  /* documentation see lib.h */

  int i, idx, dlen, len;
  int maxdlen = CAN_MAX_DLEN;
  int ret = CAN_MTU;
  unsigned char tmp;

  len = strlen(cs);
  // printf("'%s' len %d\n", cs, len);

  memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

  if (len < 4)
    return 0;

  if (cs[3] == CANID_DELIM) { /* 3 digits */
    idx = 4;
    for (i = 0; i < 3; i++) {
      if ((tmp = asc2nibble(cs[i])) > 0x0F)
        return 0;
      cf->can_id |= (tmp << (2-i)*4);
    }
  } else if (cs[8] == CANID_DELIM) { /* 8 digits */
    idx = 9;
    for (i = 0; i < 8; i++) {
      if ((tmp = asc2nibble(cs[i])) > 0x0F)
        return 0;
      cf->can_id |= (tmp << (7-i)*4);
    }
    if (!(cf->can_id & CAN_ERR_FLAG)) /* 8 digits but no errorframe?  */
      cf->can_id |= CAN_EFF_FLAG;   /* then it is an extended frame */
  } else {
    return 0;
  }
  if ((cs[idx] == 'R') || (cs[idx] == 'r')) { /* RTR frame */
    cf->can_id |= CAN_RTR_FLAG;

    /* check for optional DLC value for CAN 2.0B frames */
    if (cs[++idx] && (tmp = asc2nibble(cs[idx])) <= CAN_MAX_DLC)
      cf->len = tmp;

    return ret;
  }

  if (cs[idx] == CANID_DELIM) { /* CAN FD frame escape char '##' */
    maxdlen = CANFD_MAX_DLEN;
    ret = CANFD_MTU;

    /* CAN FD frame <canid>##<flags><data>* */
    if ((tmp = asc2nibble(cs[idx+1])) > 0x0F)
      return 0;

    cf->flags = tmp;
    idx += 2;
  }

  for (i = 0, dlen = 0; i < maxdlen; i++) {
    if (cs[idx] == DATA_SEPERATOR) /* skip (optional) separator */
      idx++;

    if (idx >= len) /* end of string => end of data */
      break;

    if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
      return 0;
    cf->data[i] = (tmp << 4);
    if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
      return 0;
    cf->data[i] |= tmp;
    dlen++;
  }
  cf->len = dlen;

  return ret;
}


int Can::demo(int argc, char ** argv)
{
  int s; /* can raw socket */
  int required_mtu;
  int mtu;
  int enable_canfd = 1;
  struct sockaddr_can addr;
  struct canfd_frame frame;
  struct ifreq ifr;

  /* check command line options */
  if (argc != 3) {
    fprintf(stderr, "Usage: %s <device> <can_frame>.\n", argv[0]);
    return 1;
  }

  /* parse CAN frame */
  required_mtu = parse_canframe(argv[2], &frame);
  if (!required_mtu) {
    fprintf(stderr, "\nWrong CAN-frame format! Try:\n\n");
    fprintf(stderr, "    <can_id>#{R|data}          for CAN 2.0 frames\n");
    fprintf(stderr, "    <can_id>##<flags>{data}    for CAN FD frames\n\n");
    fprintf(stderr, "<can_id> can have 3 (SFF) or 8 (EFF) hex chars\n");
    fprintf(stderr, "{data} has 0..8 (0..64 CAN FD) ASCII hex-values (optionally");
    fprintf(stderr, " separated by '.')\n");
    fprintf(stderr, "<flags> a single ASCII Hex value (0 .. F) which defines");
    fprintf(stderr, " canfd_frame.flags\n\n");
    fprintf(stderr, "e.g. 5A1#11.2233.44556677.88 / 123#DEADBEEF / 5AA# / ");
    fprintf(stderr, "123##1 / 213##311\n     1F334455#1122334455667788 / 123#R ");
    fprintf(stderr, "for remote transmission request.\n\n");
    return 1;
  }

  /* open socket */
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("socket");
    return 1;
  }

  strncpy(ifr.ifr_name, argv[1], IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex) {
    perror("if_nametoindex");
    return 1;
  }

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (required_mtu > CAN_MTU) {
    /* check if the frame fits into the CAN netdevice */
    if (ioctl(s, SIOCGIFMTU, &ifr) < 0) {
      perror("SIOCGIFMTU");
      return 1;
    }
    mtu = ifr.ifr_mtu;

    if (mtu != CANFD_MTU) {
      printf("CAN interface is not CAN FD capable - sorry.\n");
      return 1;
    }

    /* interface is ok - try to switch the socket into CAN FD mode */
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
             &enable_canfd, sizeof(enable_canfd))) {    // NOLINT [whitespace/braces]
      printf("error when enabling CAN FD support\n");
      return 1;
    }

    /* ensure discrete CAN FD length values 0..8, 12, 16, 20, 24, 32, 64 */
    frame.len = can_dlc2len(can_len2dlc(frame.len));
  }

  /* disable default receive filter on this RAW socket */
  /* This is obsolete as we do not read from the socket at all, but for */
  /* this reason we can remove the receive list in the Kernel to save a */
  /* little (really a very little!) CPU usage.                          */
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    return 1;
  }

  /* send frame */
  if (write(s, &frame, required_mtu) != required_mtu) {
    perror("write");
    return 1;
  }

  close(s);

  return 0;
}

}}}   // namespace hyped::utils::io

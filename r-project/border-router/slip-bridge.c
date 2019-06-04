/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \file
 *         Slip fallback interface
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Joel Hoglund <joel@sics.se>
 *         Nicolas Tsiftes <nvt@sics.se>
 */

#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "dev/slip.h"
#include "dev/uart1.h"
#include <string.h>

#define UIP_IP_BUF        ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF       ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN])

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

void set_prefix_64(uip_ipaddr_t *);

static uip_ipaddr_t last_sender;
/*---------------------------------------------------------------------------*/
static void
slip_input_callback(void)
{
 // PRINTF("SIN: %u\n", uip_len);
  if(uip_buf[0] == '!') {
    PRINTF("Got configuration message of type %c\n", uip_buf[1]);
    uip_clear_buf();
    if(uip_buf[1] == 'P') {
      uip_ipaddr_t prefix;
      /* Here we set a prefix !!! */
      memset(&prefix, 0, 16);
      memcpy(&prefix, &uip_buf[2], 8);
      PRINTF("Setting prefix ");
      PRINT6ADDR(&prefix);
      PRINTF("\n");
      set_prefix_64(&prefix);
    }
  } else if (uip_buf[0] == '?') {
    PRINTF("Got request message of type %c\n", uip_buf[1]);
    if(uip_buf[1] == 'M') {
      char* hexchar = "0123456789abcdef";
      int j;
      /* this is just a test so far... just to see if it works */
      uip_buf[0] = '!';
      for(j = 0; j < 8; j++) {
        uip_buf[2 + j * 2] = hexchar[uip_lladdr.addr[j] >> 4];
        uip_buf[3 + j * 2] = hexchar[uip_lladdr.addr[j] & 15];
      }
      uip_len = 18;
      slip_send();
      
    }
    uip_clear_buf();
  }
  /* Save the last sender received over SLIP to avoid bouncing the
     packet back if no route is found */
  uip_ipaddr_copy(&last_sender, &UIP_IP_BUF->srcipaddr);
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  slip_arch_init(BAUD2UBR(115200));
  process_start(&slip_process, NULL);
  slip_set_input_callback(slip_input_callback);
}
/*---------------------------------------------------------------------------*/
#include "core/net/mac/tsch/tsch-private.h"
extern struct tsch_asn_t tsch_current_asn;

static int
output(void)
{
  if(uip_ipaddr_cmp(&last_sender, &UIP_IP_BUF->srcipaddr)) {
    /* Do not bounce packets back over SLIP if the packet was received
       over SLIP */
    PRINTF("slip-bridge: Destination off-link but no route src=");
    PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
    PRINTF(" dst=");
    PRINT6ADDR(&UIP_IP_BUF->destipaddr);
    PRINTF("\n");
  } else {
    PRINTF("Got thing to send to PC\n");
    PRINTF(" dst=");
    PRINT6ADDR(&UIP_IP_BUF->destipaddr);
    PRINTF("\n");
    PRINTF("packet length: %d \n", UIP_IP_BUF->len[1]);

    uint8_t ndx;


    uint8_t ip_payload_length = UIP_IP_BUF->len[1];
    uint8_t coap_packet_start_location = UIP_IPH_LEN + ip_payload_length - 40;  //42 is coap payload length

    for (ndx = coap_packet_start_location; ndx < UIP_IP_BUF->len[1] + UIP_IPH_LEN; ndx++) { //to udp
        uint8_t data = ((uint8_t *) (UIP_IP_BUF))[ndx];
        PRINTF("%02x", data);
      }
      PRINTF("\n");

    uint8_t flag1 = ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location];
    uint8_t flag2 = ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 1];

    if(flag1 == 0x54 && flag2 == 0x66){

      PRINTF("Found flag: %02x %02x\n", flag1, flag2);  
      // tsch_current_asn.ls4b
      //add end_asn
      ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 11] = tsch_current_asn.ls4b & 0xff;
      ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 10] = (tsch_current_asn.ls4b >> 8) & 0xff;
      ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 9] = (tsch_current_asn.ls4b >> 16) & 0xff;
      ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 8] = (tsch_current_asn.ls4b >> 24) & 0xff;

      // memcpy(UIP_IP_BUF[coap_packet_start_location + 8], &(tsch_current_asn.ls4b), 4)

      PRINTF("Start ASN Numbers : %02x%02x%02x%02x. \n", ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 7],
                                                     ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 6],
                                                     ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 5],
                                                     ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 4]);

      PRINTF("End ASN Numbers : %02x%02x%02x%02x. \n",((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 8],
                                                     ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 9],
                                                     ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 10],
                                                     ((uint8_t *) (UIP_IP_BUF))[coap_packet_start_location + 11]);


      UIP_UDP_BUF->udpchksum = 0;
      uint16_t new_udp_checksum = ~(uip_udpchksum());
      UIP_UDP_BUF->udpchksum = new_udp_checksum;

      PRINTF("new checksum: %04x\n", new_udp_checksum);

    }


    slip_send();
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
#if !SLIP_BRIDGE_CONF_NO_PUTCHAR
#undef putchar
int
putchar(int c)
{
#define SLIP_END     0300
  static char debug_frame = 0;

  if(!debug_frame) {            /* Start of debug output */
    slip_arch_writeb(SLIP_END);
    slip_arch_writeb('\r');     /* Type debug line == '\r' */
    debug_frame = 1;
  }

  /* Need to also print '\n' because for example COOJA will not show
     any output before line end */
  slip_arch_writeb((char)c);

  /*
   * Line buffered output, a newline marks the end of debug output and
   * implicitly flushes debug output.
   */
  if(c == '\n') {
    slip_arch_writeb(SLIP_END);
    debug_frame = 0;
  }
  return c;
}
#endif
/*---------------------------------------------------------------------------*/
const struct uip_fallback_interface rpl_interface = {
  init, output
};
/*---------------------------------------------------------------------------*/
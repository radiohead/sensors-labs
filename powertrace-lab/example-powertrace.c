/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Demonstrating the powertrace application with broadcasts
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime.h"
#include "random.h"

#include "powertrace.h"

#include "dev/button-sensor.h"

#include "dev/leds.h"
#include "sys/energest.h"

#include <stdio.h>
#include <math.h>

/*---------------------------------------------------------------------------*/
PROCESS(example_broadcast_process, "BROADCAST example");
AUTOSTART_PROCESSES(&example_broadcast_process);
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  // printf("broadcast message received from %d.%d: '%s'\n", from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
static double total_energy_m = 0.0;
static unsigned int times_m = 0;

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_broadcast_process, ev, data)
{
  static struct etimer et;
  static struct etimer minute_t;
  double cpu_time = 0, lpm_time = 0, transmit_time = 0, listen_time = 0;
  double cpu_energy, lpm_energy, transmit_energy, listen_energy, total_energy;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();
  etimer_set(&minute_t, CLOCK_SECOND * 60);

  /* Start powertracing, once every two seconds. */
  // powertrace_start(CLOCK_SECOND * 2);
  
  broadcast_open(&broadcast, 129, &broadcast_call);

  while(1) {
    /* Delay 2-4 seconds */
    etimer_set(&et, CLOCK_SECOND * 4);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    packetbuf_copyfrom("Hello", 6);
    broadcast_send(&broadcast);
    printf("broadcast message sent\n");

    cpu_time = energest_type_time(ENERGEST_TYPE_CPU);
    lpm_time = energest_type_time(ENERGEST_TYPE_LPM);
    transmit_time = energest_type_time(ENERGEST_TYPE_TRANSMIT);
    listen_time = energest_type_time(ENERGEST_TYPE_LISTEN);

    cpu_energy = (cpu_time / CLOCK_SECOND) * (5 * pow(10, -3) * 3);
    lpm_energy = (lpm_time / CLOCK_SECOND) * (2.6 * pow(10, -6) * 3);
    transmit_energy = (transmit_time / CLOCK_SECOND) * (pow(10, -3) * 19.5 * 3);
    listen_energy = (listen_time / CLOCK_SECOND) * (21.8 * pow(10, -3) * 3);

    total_energy = cpu_energy + lpm_energy + transmit_energy + listen_energy;

    if (!etimer_expired(&minute_t)) {
      total_energy_m += total_energy;
      times_m += 1;
      printf("Total energy consumed per cycle: %u\n", total_energy);
    }
    else {
      printf("Average energy consumption per 4 seconds: %u\n", ((double)total_energy_m / times_m)); 
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

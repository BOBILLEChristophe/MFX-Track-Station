/*

   RailCom.h

   RailCom cutout (RCN-217) after each DCC packet, on the board proto 2:
   CUTOUT_GATE (GPIO 14) goes into the second input of the two AND gates
   (SN74HCT08N) in front of IN1 and IN2 of the L6203. At 0, both inputs are
   low: both half-bridges switch to the low side, the rails are short-
   circuited through them and a decoder can send its RailCom answer.
   R49 pulls the gate to 1: without this module, the gates are transparent.

   Timing (RCN-217): the cutout starts 26 to 32 µs after the end of the end
   bit of the packet and ends 454 to 488 µs after it. Here
   Dcc::RAILCOM_START_US = 29 and Dcc::RAILCOM_END_US = 470.

   How it is timed:
     - Dcc::encode(..., railcom = true) leaves the track still for the
       cutout after the end bit, and marks the edge that ends the end bit
       (MfxSignal::cutoutUs(), with the level the track takes on it).
     - RMTChannel::send() starts the chunk, then calls arm() with the time it
       started. The start time is known within a few µs only (the task may be
       interrupted): it is used to open a window, 40 µs before that edge.
     - The edge itself is caught by an interrupt on IN1 (the output of the
       RMT, read back on its pad), checked by its level, and a hardware timer
       (gptimer, 1 µs) then drives the gate: 0 at RAILCOM_START_US, 1 at
       RAILCOM_END_US, both minus latencyUs (latency of the two interrupts).
     - When the edge is not where it is expected (the task was late), the
       cutout of this packet is skipped and counted.

   mfx and MM: no cutout (the gate stays at 1).

   Off by default: setEnabled(true) in main.cpp, or serial command W.
   To be checked with an oscilloscope on the board: IN1_BR, IN2_BR (pins 5
   and 7 of the L6203) and the track; the cutout must start 26 to 32 µs after
   the last edge of the packet. Serial command U<µs> changes latencyUs.

*/

#pragma once

#include <Arduino.h>
#include "driver/gpio.h"

namespace RailCom
{
    // gate: CUTOUT_GATE; in1: the IN1 pin of the RMT (read back).
    // Returns false (and RailCom stays unavailable) when gate is GPIO_NUM_NC.
    bool setup(gpio_num_t gate, gpio_num_t in1);
    bool available();
    bool enabled();
    void setEnabled(bool on);

    // Called by RMTChannel::send() right after the start of a chunk that
    // holds a packet with a cutout. startUs: esp_timer at the return of
    // rmt_transmit(); cutoutUs, levelAfter: MfxSignal::cutoutUs(), cutoutLevel().
    void arm(int64_t startUs, uint32_t cutoutUs, bool levelAfter);

    // Latency of the interrupts, taken off the times of the gate (µs)
    extern volatile uint16_t latencyUs;

    // Cutouts made and skipped since start-up
    uint32_t done();
    uint32_t skipped();
}

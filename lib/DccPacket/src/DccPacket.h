/*
  DccPacket.h

  DCC packets (NMRA S-9.2, S-9.2.1) and their track signal, as a Marklin
  Gleisbox sends them (recorded with the sniffer, 11 Nov 2026):
    - half-bits of 58 µs for a 1 and 100 µs for a 0;
    - preamble of 21 bits at 1, 22 with the end bit of the previous packet
      (the sniffer counts 22 between two packets);
    - packets one after the other, without any gap: the preamble of the
      next packet starts right after the end bit;
    - speed in 28 or 128 steps (14 too), functions in groups:
      F0-F4, F5-F8, F9-F12 (F13-F20 and F21-F28 when used);
    - addresses above 127 always long (130 is sent C0 82).

  Joining the RMT chunks (MfxRMT): between two chunks the line keeps its
  level for a few tens of µs. That stretch must not fall in a 1 bit (58 µs
  halves, a decoder accepts 52 to 64). It falls in the first half of the
  start bit of the next packet: a 0 bit may be stretched (S-9.1, "zero
  stretching", up to 9.9 ms per half). A chunk then holds the end of a
  packet and the preamble of the next one:
    [second half of the start bit] bytes, end bit
    [preamble of the next packet, first half of its start bit]
  The first chunk of a series starts with the preamble; the last one ends
  with two more 1 bits, which belong to a preamble (the stretch falls in
  them, before the mfx signal).

  Pure C++, no ESP32 dependency: tested on a PC (pio test -e native).
*/

#pragma once

#include <stdint.h>
#include "MfxSignal.h"

namespace Dcc
{
    // Half-bits (µs) and preamble, as the Gleisbox
    static const uint16_t ONE_US = 58;
    static const uint16_t ZERO_US = 100;
    static const uint8_t PREAMBLE_BITS = 21; // + the end bit before: 22 ones, as recorded
    static const uint8_t TAIL_BITS = 2; // after the last packet of a series

    static const uint8_t MAX_BYTES = 6;

    struct Packet
    {
        uint8_t n;             // bytes, error byte included
        uint8_t b[MAX_BYTES];
    };

    // Speed steps of a locomotive (CS2 "Lok Datenprotokoll", system 0x05)
    enum Steps : uint8_t
    {
        STEPS_14 = 14,
        STEPS_28 = 28,
        STEPS_128 = 128
    };

    // Groups of functions, in the order of the refresh
    enum Group : uint8_t
    {
        F0_F4,
        F5_F8,
        F9_F12,
        F13_F20,
        F21_F28
    };

    // Long address (two bytes) for addr > 127, or when asked
    bool isLong(uint16_t addr, bool forceLong);

    // CS2 speed (0 to 1000) to a DCC step: 0 = stop, else 1 to 14, 28 or 126.
    // Matches the Gleisbox (recorded): 28 steps 1 + v*28/1000, 128 steps
    // 1 + v*127/1000 (the code then adds 1, as 1 is the emergency stop).
    uint8_t stepFromCs2(uint16_t v, uint8_t steps);

    // Speed and direction. step: 0 = stop, 1..max; emergency: emergency stop.
    // F0 (headlight) is carried by the speed byte in 14 steps only.
    Packet speed(uint16_t addr, bool forceLong, uint8_t steps, bool forward, uint8_t step, bool emergency,
                 bool f0 = false);
    // A group of functions (bit n of fn = Fn)
    Packet functions(uint16_t addr, bool forceLong, Group group, uint32_t fn);
    // Idle packet: FF 00 FF
    Packet idle();

    // Appends the packet to the signal. lead: the previous chunk did not end
    // in an open start bit (the preamble is sent here). more: another packet
    // follows in the next chunk (its preamble and the first half of its start
    // bit end this chunk), else the tail bits end it.
    void encode(MfxSignal &signal, const Packet &p, bool lead, bool more);
}

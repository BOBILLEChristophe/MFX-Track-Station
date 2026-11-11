/*
  MmPacket.h

  Motorola packets (Marklin MM2) and their track signal, as a Marklin
  Gleisbox sends them (recorded with the sniffer, 11 Nov 2026):

  - 18 bits, sent twice. Locomotive packet: 208 µs per bit, a 1 is a pulse
    of 182 µs then 26 µs at rest, a 0 the opposite. The track rests at the
    opposite polarity of the pulses between the packets.
  - After a packet, 1.5 ms at rest before its repetition, about 3.9 ms
    after the pair (the sniffer prints pause = 1520 to 1675 and 4088 µs,
    the last half-bit included).
  - Address: 4 trits, least significant first, 00 = 0, 11 = 1, 10 = 2
    (address 80 = 10 10 10 10, the idle address). Then F0 (11 on, 00 off),
    then 4 speed bits (code 0 = stop, 2 to 15 = steps 1 to 14), each one
    followed by a bit of the MM2 extension (direction or one function):
      direction: forward 0101 (code > 7: 0100), backward 1011 (code > 7: 1010);
      F1: 110f, F2: 001f, F3: 011f, F4: 111f, f = the state of the function,
      with two exceptions when the packet would look like a direction one
      (Rocrail ddx, comp_maerklin_2(), checked on the recorded packets).
  - Refresh of a locomotive by the Gleisbox: speed, F1, speed, F2, speed,
    F3, speed, F4, then the packet of the old function decoders: the same
    address, 11, F1 to F4 as 11 or 00, at double speed (104 µs per bit,
    91 + 13 µs, repetition 0.75 ms later).

  Pure C++, no ESP32 dependency: tested on a PC (pio test -e native).
*/

#pragma once

#include <stdint.h>
#include "MfxSignal.h"

namespace Mm
{
    // Locomotive packet: 208 µs per bit
    static const uint16_t LONG_US = 182;
    static const uint16_t SHORT_US = 26;
    // Function decoder packet: 104 µs per bit
    static const uint16_t FAST_LONG_US = 91;
    static const uint16_t FAST_SHORT_US = 13;
    // Rest between the two packets of a pair, and after the pair
    static const uint16_t PAIR_GAP_US = 1494;
    static const uint16_t FAST_PAIR_GAP_US = 745;
    static const uint16_t AFTER_US = 3900;

    static const uint8_t MAX_ADDRESS = 80;
    static const uint8_t IDLE_ADDRESS = 80;

    struct Packet
    {
        uint8_t bits[18]; // 0 or 1, in the order sent
        bool fast;        // function decoder packet (104 µs per bit)
    };

    // Level of the track during the pulses (the rest is the other one).
    // The Gleisbox pulses are at the level the sniffer prints "pol=H".
    extern bool pulseLevel;

    // CS2 speed (0 to 1000) to the code of the packet: 0 = stop, else 2..15
    uint8_t codeFromCs2(uint16_t v);

    // Speed and direction (MM2)
    Packet speed(uint8_t addr, bool f0, bool forward, uint8_t code);
    // One of F1 to F4 (MM2), with the speed code of the locomotive
    Packet function(uint8_t addr, bool f0, uint8_t code, uint8_t fn, bool on);
    // Packet of the old function decoders (F1 to F4: bits 0 to 3 of fn14)
    Packet functionDecoder(uint8_t addr, uint8_t fn14);
    // Idle packet: address 80, as the Gleisbox
    Packet idle();

    // Appends the pair (the packet twice) and the rest after it. lead: the
    // track was not at rest before (DCC, mfx): it is brought to rest and
    // held there AFTER_US first.
    void encodePair(MfxSignal &signal, const Packet &p, bool lead);
}

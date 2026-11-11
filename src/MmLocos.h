/*

   MmLocos.h

   The Motorola (MM2) locomotives driven by the master (Rocrail or an MS2),
   and their refresh on the track, as a Marklin Gleisbox does it (recorded
   with the sniffer, 11 Nov 2026, MmPacket.h):

   - every MM locomotive the master has driven stays in the refresh, even
     when it is not on the track (the Gleisbox refreshed the 15 MM
     locomotives of the MS2 list);
   - each packet is sent twice; the refresh of a locomotive goes: speed,
     F1, speed, F2, speed, F3, speed, F4, then the packet of the old
     function decoders (104 µs per bit);
   - one pair per locomotive and per round, with the DCC packets
     (DccLocos), then about 30 ms of mfx (MfxRMT).

   CS2 identifier of an MM locomotive: its address, 1 to 80 (0x0001 to
   0x0050). F0 to F4 only (MM2). The list (addresses) is kept in the NVS
   (namespace "mm"): after a restart the locomotives are refreshed at once,
   stopped, lights off, until the master drives them again. The system
   command 0x04 ("Lok Zyklus Ende") takes one out of the refresh.

   Serial command L prints them after the DCC locomotives; G swaps the
   polarity of the pulses (test: the sniffer shows pol=H on the Gleisbox).

*/

#pragma once

#include <Arduino.h>
#include "MmPacket.h"

namespace MmLocos
{
    static const uint8_t MAX_LOCOS = 32;

    // Reads the list kept in the NVS
    void setup();

    // MM2 enabled on the track (system command 0x08, bit 0): true by default
    void enable(bool on);
    bool enabled();

    // Orders of the master (address 1 to 80). The locomotive enters the
    // refresh at its first order. Each order is sent at once (urgent pair),
    // then repeated by the refresh. Return false if the list is full.
    bool speed(uint16_t addr, uint16_t cs2Speed);  // 0 to 1000
    bool direction(uint16_t addr, uint8_t cs2Dir); // 1 forward, 2 backward, 3 toggle
    bool function(uint16_t addr, uint8_t fn, bool on);
    bool emergencyStop(uint16_t addr);
    // Out of the refresh (system 0x04). Returns false if it was not there.
    bool forget(uint16_t addr);

    // State, for the answers to the requests of the master
    uint16_t cs2Speed(uint16_t addr);
    uint8_t cs2Direction(uint16_t addr); // 1 forward, 2 backward
    bool functionOn(uint16_t addr, uint8_t fn);

    uint8_t count();

    // Signal task: the packets of the next round, urgent ones first
    // (at most max, each one sent as a pair). Returns how many.
    uint8_t round(Mm::Packet *out, uint8_t max);
    bool urgent();

    void print();
}

/*

   DccLocos.h

   The DCC locomotives driven by the master (Rocrail or an MS2), and their
   refresh on the track, as a Marklin Gleisbox does it (recorded with the
   sniffer, 11 Nov 2026):

   - every DCC locomotive the master has driven stays in the refresh, even
     when it is not on the track (the Gleisbox refreshed the 10 DCC
     locomotives of the MS2 list, 3 of them on the track);
   - one packet per locomotive and per round, in turn: speed, F0-F4,
     F5-F8, F9-F12 (F13-F20 and F21-F28 too, once one of them was
     switched on);
   - a round of DCC packets, then about 30 ms for mfx (MfxRMT).

   CS2 identifier of a DCC locomotive: 0xC000 + address (1 to 10239).
   Speed steps: 28 by default, as the Gleisbox; the master sets them with
   the system command 0x05 ("Lok Datenprotokoll": 0 = 28 steps, 1 = 14,
   2 = 128, 3 = 28 with a long address, 4 = 128 with a long address).
   Addresses above 127 are always long.

   The list (address and steps) is kept in the NVS (namespace "dcc"): after
   a restart the locomotives are refreshed at once, stopped, lights off,
   until the master drives them again. The system command 0x04 ("Lok Zyklus
   Ende") takes a locomotive out of the refresh.

   Serial command L prints them after the mfx locomotives.

*/

#pragma once

#include <Arduino.h>
#include "DccPacket.h"

namespace DccLocos
{
    static const uint8_t MAX_LOCOS = 32;
    // At most this many packets in one round: about 7 ms each, the mfx
    // signal must come back every 100 ms or so (the Gleisbox: 75 ms of DCC
    // with its 10 locomotives)
    static const uint8_t MAX_ROUND = 10;
    // Time left to mfx between two rounds (Gleisbox: 30 to 31 ms)
    static const uint32_t MFX_SLOT_US = 30000;

    static const uint16_t FIRST_ID = 0xC000;
    static const uint16_t MAX_ADDRESS = 10239;

    // Reads the list kept in the NVS
    void setup();

    // DCC enabled on the track (system command 0x08, bit 2): true by default
    void enable(bool on);
    bool enabled();

    // Orders of the master (address 1 to MAX_ADDRESS). The locomotive enters
    // the refresh at its first order. Each order is sent at once (urgent
    // packet), then repeated by the refresh. Return false if the list is full.
    bool speed(uint16_t addr, uint16_t cs2Speed);   // 0 to 1000
    bool direction(uint16_t addr, uint8_t cs2Dir);  // 1 forward, 2 backward, 3 toggle
    bool function(uint16_t addr, uint8_t fn, bool on);
    bool emergencyStop(uint16_t addr);
    bool protocol(uint16_t addr, uint8_t cs2Protocol);
    // Out of the refresh (system 0x04). Returns false if it was not there.
    bool forget(uint16_t addr);

    // State, for the answers to the requests of the master
    bool known(uint16_t addr);
    uint16_t cs2Speed(uint16_t addr);
    uint8_t cs2Direction(uint16_t addr); // 1 forward, 2 backward
    bool functionOn(uint16_t addr, uint8_t fn);

    uint8_t count();

    // Signal task: the packets of the next round, urgent ones first
    // (at most max). Returns how many.
    uint8_t round(Dcc::Packet *out, uint8_t max);
    // An urgent packet is waiting (an order not sent yet)
    bool urgent();

    void print();
}

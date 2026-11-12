/*

   Accessories.h

   Accessories switched by the master (Rocrail, MS2): CS2 command 0x0B
   ("Zubehör schalten"), identifier, position, current, switch time.

   DCC accessories only for now: identifier 0x3800 + address - 1 (address
   1 to 2044, the numbers of Rocrail and the MS2). The packet of the coil
   on (DccPacket: Dcc::accessory) is sent REPEATS times, one per DCC round;
   after the switch time given by the master (unit 10 ms, 0: until its
   order "current 0"), the same packet with the coil off is sent REPEATS
   times. Not refreshed. The orders are confirmed (response bit) and the
   last position of each address is kept for the requests of the master.

   MM accessories (identifier 0x3000 + address - 1): next step.

*/

#pragma once

#include <Arduino.h>
#include "DccPacket.h"

namespace Accessories
{
    static const uint16_t FIRST_MM = 0x3000;
    static const uint16_t FIRST_DCC = 0x3800;
    static const uint8_t REPEATS = 4;

    // Order of the master. Returns false if the identifier is not handled.
    bool order(uint16_t id, uint8_t position, uint8_t current, uint16_t time10ms);
    // Last position ordered (0 or 1), for a request of the master
    uint8_t position(uint16_t id);

    // Signal task: a packet is waiting (an order, or a coil to switch off)
    bool pending(int64_t nowUs);
    // At most max DCC packets for this round
    uint8_t take(Dcc::Packet *out, uint8_t max, int64_t nowUs);
}

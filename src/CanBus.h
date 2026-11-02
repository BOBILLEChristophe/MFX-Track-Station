/*

   CanBus.h

   The Marklin CAN bus (250 kbit/s, 29-bit identifiers), as on a Gleisbox:
   an MS2 (or any other device of the CS2 protocol) can be connected to the
   command station.

   The command station is then also a gateway between the bus and Rocrail
   (TCP, port 15731), as the CS2 is:
   - a frame received on the bus goes to Rocrail, and is executed by the
     command station if it is an order (same handling as from Rocrail);
   - a frame received from Rocrail goes to the bus;
   - the answers of the command station go to both.

   Pins of the Box from Locoduino: RX GPIO 4, TX GPIO 5 (a CAN transceiver
   such as the SN65HVD230, 3.3 V). Library ACAN_ESP32 (Pierre Molinaro),
   version 1.1.x for the Arduino core 2.0.

*/

#pragma once

#include <Arduino.h>

class CanBus
{
public:
    CanBus() = delete;

    // Starts the bus. Returns false (and prints why) if the controller
    // could not start: the command station then works without the bus.
    static bool setup(gpio_num_t rx, gpio_num_t tx);
    static bool ready() { return sReady; }

    // Sends frames in the format of the TCP link (13 bytes each: identifier
    // big endian, length, 8 data bytes); len is a multiple of 13.
    static void send(const uint8_t *frames, size_t len);

    // Next frame received from the bus, in the same 13-byte format.
    // Returns false if there is none.
    static bool receive(uint8_t *frame13);

private:
    static bool sReady;
};

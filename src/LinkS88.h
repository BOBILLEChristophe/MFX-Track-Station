/*

   LinkS88.h

   Märklin Link S88 (L88, 60883) on the CAN bus, without a Central Station.

   The L88 starts in its bootloader and waits for a master. A CS2 or CS3
   wakes it up and configures it; without one (here: the command station is
   a Gleisbox for Rocrail), it stays silent. This module does what the
   program wake-up-links88 of Gerd Bertelsmann does on a Raspberry Pi
   (github.com/GBert/railroad, can2udp/src/wake-up-links88.c, links88.md):

     1. search: bootloader query (command 0x1B, no data) and ping (0x18).
        A sleeping L88 answers the bootloader query (0x1B R) with its UID,
        which starts with 53 38 ("S8"); an awake one answers the ping
        (0x18 R).
     2. wake-up of a sleeping L88: 0x1B UID E4, 0x1B UID 11, then
        0x00 UID 0C 00 nn (system command 0C: its identifier nn, here the
        last byte of its UID; this frame has the answer bit set, identifier
        0x00010300), then a ping.
     3. configuration of an awake L88 (its ping answer): one system command
        0x00 UID 0B nn vv vv per value; the L88 answers 0B nn 01 (accepted)
        or 0B nn 00 (refused):
           nn = 2, 3, 4: number of S88 modules on bus 1, 2 (RJ45, S88-N),
                         bus 3 (6 pins, old 6088), 0 to 31;
           nn = 5, 6, 7: cycle time of bus 1, 2, 3, 10 to 1000 ms.
        The frames are sent 10 ms apart, as wake-up-links88 does.

   An L88 that is switched off and on answers the bootloader query again:
   it is woken up and configured again. The search is repeated every 10 s.

   Its contacts then come as S88 events (command 0x11 R): device identifier
   (2 bytes), contact (2 bytes: 1-16 on the L88, 1001-1496 bus 1, 2001-2496
   bus 2, 3001-3496 bus 3), old state, new state, time (2 bytes). Like every
   frame of the bus, they go to Rocrail (RxTask), which shows them.

   Hardware: the L88 on the mini-DIN 7 socket J9 (board proto 2: CAN and
   ground only), with its own power supply (Märklin 66360/66361), and the
   ground of its first S88 module tied to the track rail 0 (the L88 is
   isolated).

   Number of modules and cycle time: serial command K (Discovery.cpp), kept
   in the NVS.

*/

#pragma once

#include <Arduino.h>

namespace LinkS88
{
    // Reads the configuration in the NVS. After CanBus::setup().
    void setup();

    // Every frame received from the bus (13 bytes, format of CanBus), from
    // the task RxTask.
    void onBusFrame(const uint8_t *f);

    // Search, queue of the frames to send (10 ms apart): from RxTask, at
    // every pass of its loop.
    void loop();

    // Number of S88 modules on the buses 1, 2, 3, cycle time (ms) of every
    // bus: kept in the NVS, sent to the L88 already found. From any task.
    void configure(uint8_t bus1, uint8_t bus2, uint8_t bus3, uint16_t cycleMs);

    // Configuration, L88 found, S88 events received
    void printStatus();
}

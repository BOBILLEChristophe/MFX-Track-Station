/*
  MfxFrame.h

  Builds the data frames sent to mfx decoders on the track
  (Stefan Krauss, "Schienenformat mfx", §2.2 and §3).

  Pure C++, no ESP32 dependency: it can be tested on a PC
  (pio test -e native).

  A frame is a list of bits, most significant bit first:
    address + command(s) + 8-bit CRC.
  Synchronisation patterns and bit stuffing are added by the track
  signal generator (MfxRMT), not here.

  Usage:
    MfxFrame f;
    f.address(5);
    f.drive(false, 64);
    f.functions16(0x0001);
    f.finish();               // appends the CRC
    f.toBuffer(buff, sizeof(buff));

  mfx is a registered trademark of Gebr. Marklin & Cie GmbH.
*/

#pragma once

#include <stddef.h>
#include <stdint.h>

class MfxFrame
{
public:
    static const uint8_t MAX_BITS = 128;

    MfxFrame() { clear(); }

    void clear();

    // Rail address (SID). Uses the shortest format (Krauss 2.2.4):
    //   10 + 7 bits, 110 + 9 bits, 1110 + 11 bits, 1111 + 14 bits.
    void address(uint16_t sid);

    // Broadcast address 0 (7-bit format), used before a decoder has a SID.
    void broadcast() { address(0); }

    // 001 R SSSSSSS : drive. reverse = R, speed 0..127 (1 = emergency stop).
    void drive(bool reverse, uint8_t speed);

    // 0111 + F15..F0 : functions F0 to F15, bit n of mask = Fn.
    void functions16(uint16_t mask);

    // 100 NNNNNNN 0 F : one function, F0 to F127 (Krauss 3.1.6). The other
    // functions keep their state: used for F16 and beyond.
    void function(uint8_t number, bool on);

    // 111 011 + 14-bit SID + 32-bit UID : assignment of a rail address (Bind).
    // Sent to the broadcast address.
    void bind(uint16_t sid, uint32_t uid);

    // 111 100 + 32-bit UID : existence query (ping, mfx verify, Krauss
    // 3.2.5), sent to the rail address of the decoder. It answers with a
    // 1-bit feedback if it has this UID. Beware: a wrong UID makes the
    // decoder switch off its outputs and register again.
    void verify(uint32_t uid);

    // 111 010 + 6-bit count + 32-bit UID : decoder search (Krauss 3.2.3).
    // Sent to the broadcast address. The decoders not yet registered whose
    // UID starts with the first `count` bits of `uid` answer with a 1-bit
    // feedback. count = 0: every decoder not yet registered answers.
    void search(uint8_t count, uint32_t uid);

    // 111 000 + CV number (10 bits) + index (6 bits) + CC (2 bits) : read
    // configuration variables (Krauss 3.2.1), answered by a data feedback.
    // bytes = 1, 2, 4 or 8 (CC = 00, 01, 10, 11): the CVs cv/index,
    // cv/index+1, ... are sent back. Any other value reads 1 byte.
    void readCV(uint16_t cv, uint8_t index, uint8_t bytes);

    // Value of CC for a number of bytes (1, 2, 4, 8), and the reverse
    static uint8_t countCode(uint8_t bytes);
    static uint8_t countBytes(uint8_t code) { return (uint8_t)(1 << (code & 3)); }

    // 111 101 + 32-bit UID + 16-bit counter : command station broadcast.
    // Sent to the broadcast address.
    void centrale(uint32_t uid, uint16_t counter);

    // Appends the 8-bit CRC. Call once, after the last command.
    void finish();

    // Number of bits, and bit i (0 = first bit sent). Not named bit(): Arduino.h defines a bit() macro.
    uint8_t size() const { return mSize; }
    bool bitAt(uint8_t i) const;

    // True if a command did not fit in MAX_BITS bits.
    bool overflow() const { return mOverflow; }

    // Copies the frame in the format used by MfxRMT:
    // buff[0] = number of bits, buff[1..n] = one bit per byte (0 or 1).
    // Returns false if the buffer is too small.
    bool toBuffer(uint8_t *buff, size_t buffSize) const;

    // Is the frame in buff (format of toBuffer()) a bind command? The
    // Gleisbox holds the track still for 1.6 ms after each bind.
    static bool isBind(const uint8_t *buff);

    // CRC of the first nbits bits (Krauss: start value 0x7F, polynomial
    // x^8 + x^2 + x + 1, computed over the bits followed by 8 zero bits).
    static uint8_t crc(const MfxFrame &frame, uint8_t nbits);

private:
    void put(uint32_t value, uint8_t nbits); // most significant bit first

    uint8_t mBits[MAX_BITS / 8];
    uint8_t mSize;
    bool mOverflow;
};

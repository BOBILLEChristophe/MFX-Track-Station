/*
  MfxFrame.cpp
*/

#include "MfxFrame.h"

#include <string.h>

void MfxFrame::clear()
{
    memset(mBits, 0, sizeof(mBits));
    mSize = 0;
    mOverflow = false;
}

void MfxFrame::put(uint32_t value, uint8_t nbits)
{
    for (int8_t b = nbits - 1; b >= 0; b--)
    {
        if (mSize >= MAX_BITS)
        {
            mOverflow = true;
            return;
        }
        if ((value >> b) & 1UL)
            mBits[mSize / 8] |= (uint8_t)(0x80 >> (mSize % 8));
        mSize++;
    }
}

bool MfxFrame::bitAt(uint8_t i) const
{
    if (i >= mSize)
        return false;
    return (mBits[i / 8] >> (7 - i % 8)) & 1;
}

void MfxFrame::address(uint16_t sid)
{
    if (sid < (1u << 7))
    {
        put(0b10, 2);
        put(sid, 7);
    }
    else if (sid < (1u << 9))
    {
        put(0b110, 3);
        put(sid, 9);
    }
    else if (sid < (1u << 11))
    {
        put(0b1110, 4);
        put(sid, 11);
    }
    else
    {
        put(0b1111, 4);
        put(sid & 0x3FFF, 14);
    }
}

void MfxFrame::drive(bool reverse, uint8_t speed)
{
    put(0b001, 3);
    put(reverse ? 1 : 0, 1);
    put(speed & 0x7F, 7);
}

void MfxFrame::functions16(uint16_t mask)
{
    put(0b0111, 4);
    put(mask, 16); // F15 first, F0 last
}

void MfxFrame::function(uint8_t number, bool on)
{
    put(0b100, 3);
    put(number & 0x7F, 7);
    put(0, 1);
    put(on ? 1 : 0, 1);
}

void MfxFrame::verify(uint32_t uid)
{
    put(0b111100, 6);
    put(uid, 32);
}

void MfxFrame::bind(uint16_t sid, uint32_t uid)
{
    put(0b111011, 6);
    put(sid & 0x3FFF, 14);
    put(uid, 32);
}

uint8_t MfxFrame::countCode(uint8_t bytes)
{
    switch (bytes)
    {
    case 2:
        return 1;
    case 4:
        return 2;
    case 8:
        return 3;
    default:
        return 0;
    }
}

void MfxFrame::readCV(uint16_t cv, uint8_t index, uint8_t bytes)
{
    put(0b111000, 6);
    put(cv & 0x3FF, 10);
    put(index & 0x3F, 6);
    put(countCode(bytes), 2);
}

void MfxFrame::search(uint8_t count, uint32_t uid)
{
    put(0b111010, 6);
    put(count & 0x3F, 6);
    put(uid, 32);
}

void MfxFrame::centrale(uint32_t uid, uint16_t counter)
{
    put(0b111101, 6);
    put(uid, 32);
    put(counter, 16);
}

uint8_t MfxFrame::crc(const MfxFrame &frame, uint8_t nbits)
{
    uint16_t c = 0x007F;
    for (uint8_t i = 0; i < nbits + 8; i++)
    {
        const bool b = i < nbits ? frame.bitAt(i) : false;
        c = (c << 1) + b;
        if (c & 0x0100)
            c = (c & 0x00FF) ^ 0x07;
    }
    return (uint8_t)c;
}

void MfxFrame::finish()
{
    put(crc(*this, mSize), 8);
}

bool MfxFrame::isBind(const uint8_t *buff)
{
    const uint8_t n = buff[0];
    // Address: 10 + 7 bits, 110 + 9, 1110 + 11, 1111 + 14 (address())
    uint8_t ones = 0;
    while (ones < 4 && ones + 1 <= n && buff[1 + ones])
        ones++;
    static const uint8_t lengths[5] = {0, 2 + 7, 3 + 9, 4 + 11, 4 + 14};
    if (ones == 0)
        return false;
    const uint8_t cmd = lengths[ones]; // index of the first command bit (0-based)
    static const uint8_t code[6] = {1, 1, 1, 0, 1, 1};
    if (cmd + 6 > n)
        return false;
    for (uint8_t i = 0; i < 6; i++)
        if (buff[1 + cmd + i] != code[i])
            return false;
    return true;
}

bool MfxFrame::toBuffer(uint8_t *buff, size_t buffSize) const
{
    if (buffSize < (size_t)mSize + 1)
        return false;
    buff[0] = mSize;
    for (uint8_t i = 0; i < mSize; i++)
        buff[i + 1] = bitAt(i) ? 1 : 0;
    return true;
}

/*
  DccPacket.cpp
*/

#include "DccPacket.h"

namespace Dcc
{
    bool isLong(uint16_t addr, bool forceLong)
    {
        return forceLong || addr > 127;
    }

    uint8_t stepFromCs2(uint16_t v, uint8_t steps)
    {
        if (v == 0)
            return 0;
        if (v > 1000)
            v = 1000;
        uint8_t max;
        uint32_t step;
        switch (steps)
        {
        case STEPS_14:
            max = 14;
            step = 1 + (uint32_t)v * 14 / 1000;
            break;
        case STEPS_128:
            max = 126;
            step = 1 + (uint32_t)v * 127 / 1000;
            break;
        default:
            max = 28;
            step = 1 + (uint32_t)v * 28 / 1000;
            break;
        }
        return step > max ? max : (uint8_t)step;
    }

    // Address bytes, then the error byte is added by finish()
    static void start(Packet &p, uint16_t addr, bool forceLong)
    {
        p.n = 0;
        if (isLong(addr, forceLong))
        {
            p.b[p.n++] = (uint8_t)(0xC0 | ((addr >> 8) & 0x3F));
            p.b[p.n++] = (uint8_t)addr;
        }
        else
            p.b[p.n++] = (uint8_t)(addr & 0x7F);
    }

    static void finish(Packet &p)
    {
        uint8_t x = 0;
        for (uint8_t i = 0; i < p.n; i++)
            x ^= p.b[i];
        p.b[p.n++] = x;
    }

    Packet speed(uint16_t addr, bool forceLong, uint8_t steps, bool forward, uint8_t step, bool emergency, bool f0)
    {
        Packet p;
        start(p, addr, forceLong);
        const uint8_t dir = forward ? 1 : 0;
        if (steps == STEPS_128)
        {
            // 0011 1111, then D SSSSSSS: 0 stop, 1 emergency stop, 2..127
            uint8_t code = emergency ? 1 : (step == 0 ? 0 : (uint8_t)(step > 126 ? 127 : step + 1));
            p.b[p.n++] = 0x3F;
            p.b[p.n++] = (uint8_t)(dir << 7 | code);
        }
        else if (steps == STEPS_14)
        {
            // 01DC SSSS, C = F0: 0 stop, 1 emergency stop, 2..15
            uint8_t code = emergency ? 1 : (step == 0 ? 0 : (uint8_t)(step > 14 ? 15 : step + 1));
            p.b[p.n++] = (uint8_t)(0x40 | dir << 5 | (f0 ? 0x10 : 0) | code);
        }
        else
        {
            // 01DC SSSS, the 5-bit value is SSSS C: 0 stop, 2 emergency stop
            // (1 and 3 too), 4..31 for the steps 1 to 28
            uint8_t v = emergency ? 2 : (step == 0 ? 0 : (uint8_t)((step > 28 ? 28 : step) + 3));
            p.b[p.n++] = (uint8_t)(0x40 | dir << 5 | (v & 1) << 4 | v >> 1);
        }
        finish(p);
        return p;
    }

    Packet functions(uint16_t addr, bool forceLong, Group group, uint32_t fn)
    {
        Packet p;
        start(p, addr, forceLong);
        switch (group)
        {
        case F0_F4: // 100 F0 F4 F3 F2 F1
            p.b[p.n++] = (uint8_t)(0x80 | (fn & 1) << 4 | (fn >> 1 & 0x0F));
            break;
        case F5_F8: // 1011 F8 F7 F6 F5
            p.b[p.n++] = (uint8_t)(0xB0 | (fn >> 5 & 0x0F));
            break;
        case F9_F12: // 1010 F12 F11 F10 F9
            p.b[p.n++] = (uint8_t)(0xA0 | (fn >> 9 & 0x0F));
            break;
        case F13_F20: // 1101 1110, F20..F13
            p.b[p.n++] = 0xDE;
            p.b[p.n++] = (uint8_t)(fn >> 13);
            break;
        case F21_F28: // 1101 1111, F28..F21
            p.b[p.n++] = 0xDF;
            p.b[p.n++] = (uint8_t)(fn >> 21);
            break;
        }
        finish(p);
        return p;
    }

    Packet idle()
    {
        Packet p;
        p.n = 3;
        p.b[0] = 0xFF;
        p.b[1] = 0x00;
        p.b[2] = 0xFF;
        return p;
    }

    Packet accessory(uint16_t address, bool position, bool on)
    {
        Packet p;
        const uint16_t a = (uint16_t)(address + 3); // address 1: decoder 1, pair 0
        const uint16_t decoder = (uint16_t)(a >> 2);
        const uint8_t pair = (uint8_t)(a & 3);
        p.n = 0;
        p.b[p.n++] = (uint8_t)(0x80 | (decoder & 0x3F));
        p.b[p.n++] = (uint8_t)(0x80 | ((~decoder >> 6) & 0x07) << 4 | (on ? 0x08 : 0) | pair << 1 | (position ? 1 : 0));
        finish(p);
        return p;
    }

    static void bit(MfxSignal &s, bool one)
    {
        const uint16_t us = one ? ONE_US : ZERO_US;
        s.half(us);
        s.half(us);
    }

    void encode(MfxSignal &s, const Packet &p, bool lead, bool more)
    {
        if (lead)
        {
            for (uint8_t i = 0; i < PREAMBLE_BITS; i++)
                bit(s, true);
            s.half(ZERO_US); // first half of the start bit
        }
        s.half(ZERO_US); // second half of the start bit
        for (uint8_t i = 0; i < p.n; i++)
        {
            for (int8_t k = 7; k >= 0; k--)
                bit(s, (p.b[i] >> k) & 1);
            bit(s, i + 1 == p.n); // separator 0, or the end bit 1
        }
        if (more)
        {
            for (uint8_t i = 0; i < PREAMBLE_BITS; i++)
                bit(s, true);
            s.half(ZERO_US); // the stretch between the chunks falls here
        }
        else
            for (uint8_t i = 0; i < TAIL_BITS; i++)
                bit(s, true);
    }
}

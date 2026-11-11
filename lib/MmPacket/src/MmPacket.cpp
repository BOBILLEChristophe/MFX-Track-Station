/*
  MmPacket.cpp
*/

#include "MmPacket.h"

namespace Mm
{
    bool pulseLevel = true;

    uint8_t codeFromCs2(uint16_t v)
    {
        if (v == 0)
            return 0;
        if (v > 1000)
            v = 1000;
        uint32_t step = 1 + (uint32_t)v * 14 / 1000; // 1 to 14, as the DCC steps
        if (step > 14)
            step = 14;
        return (uint8_t)(step + 1); // code 1 is the old reversing command
    }

    // Address and F0: bits 0 to 9
    static void head(Packet &p, uint8_t addr, bool f0)
    {
        p.fast = false;
        uint8_t a = addr > MAX_ADDRESS ? IDLE_ADDRESS : addr;
        for (uint8_t i = 0; i < 4; i++)
        {
            const uint8_t t = a % 3;
            a /= 3;
            p.bits[2 * i] = t != 0;     // 0: 00, 1: 11, 2: 10
            p.bits[2 * i + 1] = t == 1;
        }
        p.bits[8] = p.bits[9] = f0 ? 1 : 0;
    }

    // Speed bits (A C E G, least significant first) and the MM2 bits (B D F H)
    static void data(Packet &p, uint8_t code, const uint8_t mm2[4])
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            p.bits[10 + 2 * i] = (code >> i) & 1;
            p.bits[11 + 2 * i] = mm2[i];
        }
    }

    Packet speed(uint8_t addr, bool f0, bool forward, uint8_t code)
    {
        Packet p;
        head(p, addr, f0);
        static const uint8_t fwdLow[4] = {0, 1, 0, 1}, fwdHigh[4] = {0, 1, 0, 0};
        static const uint8_t bwdLow[4] = {1, 0, 1, 1}, bwdHigh[4] = {1, 0, 1, 0};
        data(p, code, forward ? (code > 7 ? fwdHigh : fwdLow) : (code > 7 ? bwdHigh : bwdLow));
        return p;
    }

    Packet function(uint8_t addr, bool f0, uint8_t code, uint8_t fn, bool on)
    {
        Packet p;
        head(p, addr, f0);
        uint8_t m[4];
        switch (fn)
        {
        case 1:
            m[0] = 1, m[1] = 1, m[2] = 0;
            break;
        case 2:
            m[0] = 0, m[1] = 0, m[2] = 1;
            break;
        case 3:
            m[0] = 0, m[1] = 1, m[2] = 1;
            break;
        default:
            m[0] = 1, m[1] = 1, m[2] = 1;
            fn = 4;
            break;
        }
        m[3] = on ? 1 : 0;
        // The packet must not read as a direction packet
        static const uint8_t offCode[5] = {0, 3, 4, 6, 7}, onCode[5] = {0, 11, 12, 14, 15};
        if (!on && code == offCode[fn])
            m[0] = 1, m[1] = 0, m[2] = 1;
        if (on && code == onCode[fn])
            m[0] = 0, m[1] = 1, m[2] = 0;
        data(p, code, m);
        return p;
    }

    Packet functionDecoder(uint8_t addr, uint8_t fn14)
    {
        Packet p;
        head(p, addr, true); // second trit: always 11
        for (uint8_t i = 0; i < 4; i++)
            p.bits[10 + 2 * i] = p.bits[11 + 2 * i] = (fn14 >> i) & 1;
        p.fast = true;
        return p;
    }

    Packet idle()
    {
        Packet p;
        head(p, IDLE_ADDRESS, false);
        static const uint8_t open[4] = {0, 0, 0, 0};
        data(p, 0x0F, open); // 10 10 10 10, as recorded
        return p;
    }

    static void packet(MfxSignal &s, const Packet &p)
    {
        const uint16_t l = p.fast ? FAST_LONG_US : LONG_US;
        const uint16_t sh = p.fast ? FAST_SHORT_US : SHORT_US;
        for (uint8_t i = 0; i < 18; i++)
        {
            // From rest: a pulse, then back to rest
            s.half(p.bits[i] ? l : sh);
            s.half(p.bits[i] ? sh : l);
        }
    }

    void encodePair(MfxSignal &s, const Packet &p, bool lead)
    {
        if (lead)
        {
            if (s.level() == pulseLevel)
                s.half(AFTER_US); // to rest, held there
            else
                s.hold(AFTER_US);
        }
        packet(s, p);
        s.hold(p.fast ? FAST_PAIR_GAP_US : PAIR_GAP_US);
        packet(s, p);
        s.hold(AFTER_US);
    }
}

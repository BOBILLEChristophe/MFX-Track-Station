/*
  Unit tests of MmPacket, run on a PC:  pio test -e native

  The expected packets are those of a Marklin Gleisbox driven by an MS2,
  recorded on the track with the sniffer (11 Nov 2026).
*/

#include <unity.h>
#include <string.h>
#include "MmPacket.h"

void setUp() {}
void tearDown() {}

static void check(const Mm::Packet &p, const char *want)
{
    char got[19];
    for (uint8_t i = 0; i < 18; i++)
        got[i] = p.bits[i] ? '1' : '0';
    got[18] = 0;
    TEST_ASSERT_EQUAL_STRING(want, got);
}

void test_idle()
{
    check(Mm::idle(), "101010100010101010");
}

void test_speed()
{
    // Loco 60, recorded: stopped backward and forward, F0 on; backward code 14
    check(Mm::speed(60, true, false, 0), "001000101101000101");
    check(Mm::speed(60, true, true, 0), "001000101100010001");
    check(Mm::speed(60, true, false, 14), "001000101101101110");
    // Loco 72, forward, code 3
    check(Mm::speed(72, false, true, 3), "000010100010110001");
}

void test_functions()
{
    // Loco 72, code 3, F4 off (recorded)
    check(Mm::function(72, false, 3, 4, false), "000010100011110100");
    // Exception: F1 off at code 3 reads 101 instead of 110
    check(Mm::function(72, false, 3, 1, false), "000010100011100100");
    // F2 off at code 3 (recorded)
    check(Mm::function(72, false, 3, 2, false), "000010100010100100");
}

void test_function_decoder()
{
    check(Mm::functionDecoder(72, 0), "000010101100000000");
    check(Mm::functionDecoder(60, 0x0F), "001000101111111111");
    TEST_ASSERT_TRUE(Mm::functionDecoder(72, 0).fast);
}

void test_codes()
{
    TEST_ASSERT_EQUAL_UINT8(0, Mm::codeFromCs2(0));
    TEST_ASSERT_EQUAL_UINT8(2, Mm::codeFromCs2(1));
    TEST_ASSERT_EQUAL_UINT8(15, Mm::codeFromCs2(1000));
}

// The pulses are at pulseLevel, 182/26 µs, the rest at the other level
void test_signal()
{
    static MfxSignal s;
    s.clear();
    s.setLevel(Mm::pulseLevel); // e.g. after DCC
    const Mm::Packet p = Mm::speed(60, true, false, 2);
    Mm::encodePair(s, p, true);
    uint16_t i = 0;
    // Lead: to rest, held
    TEST_ASSERT_EQUAL(!Mm::pulseLevel, s.item(i).level0);
    TEST_ASSERT_EQUAL_UINT32(Mm::AFTER_US, s.item(i).duration0 + s.item(i).duration1);
    i++;
    for (uint8_t rep = 0; rep < 2; rep++)
    {
        for (uint8_t b = 0; b < 18; b++)
        {
            const MfxItem &a = s.item(i++), &r = s.item(i++);
            TEST_ASSERT_EQUAL(Mm::pulseLevel, a.level0);
            TEST_ASSERT_EQUAL(!Mm::pulseLevel, r.level0);
            TEST_ASSERT_EQUAL_UINT32(p.bits[b] ? 182 : 26, a.duration0 + a.duration1);
            TEST_ASSERT_EQUAL_UINT32(p.bits[b] ? 26 : 182, r.duration0 + r.duration1);
        }
        const MfxItem &g = s.item(i++);
        TEST_ASSERT_EQUAL(!Mm::pulseLevel, g.level0);
        TEST_ASSERT_EQUAL_UINT32(rep ? Mm::AFTER_US : Mm::PAIR_GAP_US, g.duration0 + g.duration1);
    }
    TEST_ASSERT_EQUAL_UINT16(i, s.count());
    TEST_ASSERT_EQUAL(!Mm::pulseLevel, s.level());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_idle);
    RUN_TEST(test_speed);
    RUN_TEST(test_functions);
    RUN_TEST(test_function_decoder);
    RUN_TEST(test_codes);
    RUN_TEST(test_signal);
    return UNITY_END();
}

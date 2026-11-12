/*
  Unit tests of DccPacket, run on a PC:  pio test -e native

  The expected packets are those of a Marklin Gleisbox driven by an MS2,
  recorded on the track with the sniffer (11 Nov 2026).
*/

#include <unity.h>
#include <string.h>
#include "DccPacket.h"

void setUp() {}
void tearDown() {}

static void check(const Dcc::Packet &p, const uint8_t *want, uint8_t n)
{
    TEST_ASSERT_EQUAL_UINT8(n, p.n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(want, p.b, n);
}

void test_speed_28()
{
    const uint8_t a[] = {0x03, 0x60, 0x63}; // loco 3 stopped, forward
    check(Dcc::speed(3, false, Dcc::STEPS_28, true, 0, false), a, 3);
    const uint8_t b[] = {0x08, 0x7A, 0x72}; // loco 8, step 18
    check(Dcc::speed(8, false, Dcc::STEPS_28, true, 18, false), b, 3);
    const uint8_t c[] = {0x0C, 0x76, 0x7A}; // loco 12, step 10
    check(Dcc::speed(12, false, Dcc::STEPS_28, true, 10, false), c, 3);
    const uint8_t d[] = {0x22, 0x60, 0x42};
    check(Dcc::speed(34, false, Dcc::STEPS_28, true, 0, false), d, 3);
}

void test_speed_128_long()
{
    const uint8_t a[] = {0xD9, 0x00, 0x3F, 0x80, 0x66}; // 6400 stopped
    check(Dcc::speed(6400, false, Dcc::STEPS_128, true, 0, false), a, 5);
    const uint8_t b[] = {0xD9, 0x00, 0x3F, 0x91, 0x77}; // 6400, step 16
    check(Dcc::speed(6400, false, Dcc::STEPS_128, true, 16, false), b, 5);
    const uint8_t c[] = {0x08, 0x3F, 0x80, 0xB7};
    check(Dcc::speed(8, false, Dcc::STEPS_128, true, 0, false), c, 4);
    const uint8_t e[] = {0xD9, 0x00, 0x3F, 0x01, 0xE7}; // emergency stop, backward
    check(Dcc::speed(6400, false, Dcc::STEPS_128, false, 0, true), e, 5);
}

void test_functions()
{
    const uint8_t a[] = {0x03, 0x9F, 0x9C}; // F0 to F4 on
    check(Dcc::functions(3, false, Dcc::F0_F4, 0x1F), a, 3);
    const uint8_t b[] = {0x03, 0xB7, 0xB4}; // F5, F6, F7 on
    check(Dcc::functions(3, false, Dcc::F5_F8, 0xE0), b, 3);
    const uint8_t c[] = {0x03, 0xA0, 0xA3};
    check(Dcc::functions(3, false, Dcc::F9_F12, 0xE0), c, 3);
    const uint8_t d[] = {0xC0, 0x82, 0xA0, 0xE2}; // 130: long address
    check(Dcc::functions(130, false, Dcc::F9_F12, 0), d, 4);
    const uint8_t e[] = {0xD9, 0x00, 0x91, 0x48}; // 6400: F0 and F1
    check(Dcc::functions(6400, false, Dcc::F0_F4, 0x03), e, 4);
    const uint8_t f[] = {0x03, 0xDE, 0x81, 0x5C}; // F13 and F20
    check(Dcc::functions(3, false, Dcc::F13_F20, (1u << 13) | (1u << 20)), f, 4);
}

void test_idle()
{
    const uint8_t a[] = {0xFF, 0x00, 0xFF};
    check(Dcc::idle(), a, 3);
}

void test_accessory()
{
    // Address 1: decoder 1, pair 0, position 1, coil on: 81 F9
    const uint8_t a[] = {0x81, 0xF9, 0x78};
    check(Dcc::accessory(1, true, true), a, 3);
    // Address 4: decoder 1, pair 3; address 5: decoder 2, pair 0
    const uint8_t b[] = {0x81, 0xFE, 0x7F};
    check(Dcc::accessory(4, false, true), b, 3);
    const uint8_t c[] = {0x82, 0xF8, 0x7A};
    check(Dcc::accessory(5, false, true), c, 3);
    // Address 150: decoder 38 (0x26), pair 1; coil off
    const uint8_t d[] = {0xA6, 0xF3, 0x55};
    check(Dcc::accessory(150, true, false), d, 3);
    // Address 2044: decoder 511 (high bits 111, inverted 000), pair 3
    const uint8_t e[] = {0xBF, 0x8F, 0x30};
    check(Dcc::accessory(2044, true, true), e, 3);
}

void test_steps_from_cs2()
{
    // Recorded: CAN speed -> 128-step code (step + 1)
    TEST_ASSERT_EQUAL_UINT8(0, Dcc::stepFromCs2(0, Dcc::STEPS_128));
    TEST_ASSERT_EQUAL_UINT8(31 - 1, Dcc::stepFromCs2(235, Dcc::STEPS_128));
    TEST_ASSERT_EQUAL_UINT8(37 - 1, Dcc::stepFromCs2(277, Dcc::STEPS_128));
    TEST_ASSERT_EQUAL_UINT8(29 - 1, Dcc::stepFromCs2(217, Dcc::STEPS_128));
    TEST_ASSERT_EQUAL_UINT8(25 - 1, Dcc::stepFromCs2(187, Dcc::STEPS_128));
    TEST_ASSERT_EQUAL_UINT8(126, Dcc::stepFromCs2(1000, Dcc::STEPS_128));
    // Recorded: CAN speed -> 28-step
    TEST_ASSERT_EQUAL_UINT8(1, Dcc::stepFromCs2(1, Dcc::STEPS_28));
    TEST_ASSERT_EQUAL_UINT8(12, Dcc::stepFromCs2(421, Dcc::STEPS_28));
    TEST_ASSERT_EQUAL_UINT8(15, Dcc::stepFromCs2(529, Dcc::STEPS_28));
    TEST_ASSERT_EQUAL_UINT8(17, Dcc::stepFromCs2(589, Dcc::STEPS_28));
    TEST_ASSERT_EQUAL_UINT8(28, Dcc::stepFromCs2(1000, Dcc::STEPS_28));
}

// ---------------------------------------------------------------------------
//  Decoder of the signal: half-bits from the items, then packets
// ---------------------------------------------------------------------------

static uint16_t halves[4000];
static uint16_t nHalves;

// Each item must be one half-bit: one level, a transition at its start
static void collect(const MfxSignal &s, bool *level)
{
    for (uint16_t i = 0; i < s.count(); i++)
    {
        const MfxItem &it = s.item(i);
        TEST_ASSERT_EQUAL(it.level0, it.level1);
        TEST_ASSERT_NOT_EQUAL(*level, it.level0);
        *level = it.level0;
        halves[nHalves++] = (uint16_t)(it.duration0 + it.duration1);
    }
}

// Packets found in halves[]: preamble of at least 10 ones, start bit, bytes
static uint8_t decode(Dcc::Packet *out, uint8_t max, uint8_t *preambles)
{
    uint8_t found = 0;
    uint16_t i = 0;
    while (i + 1 < nHalves && found < max)
    {
        uint8_t ones = 0;
        while (i + 1 < nHalves && halves[i] == Dcc::ONE_US && halves[i + 1] == Dcc::ONE_US)
        {
            ones++;
            i += 2;
        }
        if (i + 1 >= nHalves)
            break;
        TEST_ASSERT_EQUAL_UINT16(Dcc::ZERO_US, halves[i]); // start bit
        TEST_ASSERT_EQUAL_UINT16(Dcc::ZERO_US, halves[i + 1]);
        i += 2;
        TEST_ASSERT_TRUE(ones >= 10);
        preambles[found] = ones;
        Dcc::Packet &p = out[found];
        p.n = 0;
        for (;;)
        {
            uint8_t b = 0;
            for (uint8_t k = 0; k < 8; k++)
            {
                TEST_ASSERT_EQUAL_UINT16(halves[i], halves[i + 1]);
                b = (uint8_t)(b << 1 | (halves[i] == Dcc::ONE_US ? 1 : 0));
                i += 2;
            }
            p.b[p.n++] = b;
            TEST_ASSERT_EQUAL_UINT16(halves[i], halves[i + 1]);
            const bool end = halves[i] == Dcc::ONE_US;
            if (end)
                break; // the end bit is also the first bit of the next preamble
            i += 2;
        }
        found++;
    }
    return found;
}

void test_encode_series()
{
    const Dcc::Packet p[3] = {Dcc::speed(3, false, Dcc::STEPS_28, true, 0, false),
                              Dcc::functions(8, false, Dcc::F0_F4, 1),
                              Dcc::speed(6400, false, Dcc::STEPS_128, true, 16, false)};
    static MfxSignal s;
    bool level = false;
    nHalves = 0;
    for (uint8_t k = 0; k < 3; k++)
    {
        s.clear();
        Dcc::encode(s, p[k], k == 0, k < 2);
        // A chunk followed by another one ends in the first half of a start bit
        if (k < 2)
        {
            const MfxItem &last = s.item(s.count() - 1);
            TEST_ASSERT_EQUAL_UINT16(Dcc::ZERO_US, last.duration0 + last.duration1);
        }
        collect(s, &level);
    }
    Dcc::Packet got[4];
    uint8_t pre[4];
    TEST_ASSERT_EQUAL_UINT8(3, decode(got, 4, pre));
    for (uint8_t k = 0; k < 3; k++)
    {
        TEST_ASSERT_EQUAL_UINT8(p[k].n, got[k].n);
        TEST_ASSERT_EQUAL_UINT8_ARRAY(p[k].b, got[k].b, p[k].n);
    }
    // Preambles: 21 ones, 22 with the end bit of the previous packet (as
    // recorded on the Gleisbox)
    TEST_ASSERT_EQUAL_UINT8(21, pre[0]);
    TEST_ASSERT_EQUAL_UINT8(22, pre[1]);
    TEST_ASSERT_EQUAL_UINT8(22, pre[2]);
    // The series ends with the end bit and two more ones
    TEST_ASSERT_EQUAL_UINT16(Dcc::ONE_US, halves[nHalves - 1]);
    TEST_ASSERT_EQUAL_UINT16(Dcc::ONE_US, halves[nHalves - 6]);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_speed_28);
    RUN_TEST(test_speed_128_long);
    RUN_TEST(test_functions);
    RUN_TEST(test_idle);
    RUN_TEST(test_accessory);
    RUN_TEST(test_steps_from_cs2);
    RUN_TEST(test_encode_series);
    return UNITY_END();
}

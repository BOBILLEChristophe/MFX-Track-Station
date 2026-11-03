/*
  Unit tests of MfxFrame, run on a PC:  pio test -e native

  The reference functions below are copied from the frame building code
  of version 0.10.0 (Message.cpp). MfxFrame must produce exactly the same
  bits, so the track signal does not change.
*/

#include <unity.h>
#include <string.h>
#include "MfxFrame.h"

// ---------------------------------------------------------------------------
//  Reference: frame building of version 0.10.0 (one bit per byte, buff[0] = length)
// ---------------------------------------------------------------------------

static uint8_t ref[100];
static uint8_t refLen;
static uint16_t refCrc;

static void refAddr0()
{
    ref[1] = 1;
    ref[2] = 0;
    refLen = 2;
    for (uint8_t a = 0; a < 7; a++)
        ref[++refLen] = 0;
}

static void refAddr(uint8_t address)
{
    ref[1] = 1;
    ref[2] = 0;
    refLen = 2;
    for (uint8_t a = 0, b = 6; a < 7; a++, b--)
        ref[++refLen] = (address & (1 << b)) >> b;
}

static void refBCRC(bool b)
{
    refCrc = (refCrc << 1) + b;
    if ((refCrc & 0x0100) > 0)
        refCrc = (refCrc & 0x00FF) ^ 0x07;
}

static void refCRC()
{
    uint8_t a, b;
    refCrc = 0x007F;
    for (a = 1; a < ref[0] + 1; a++)
        refBCRC(ref[a]);
    for (a = 0; a < 8; a++)
        refBCRC(0);
    b = 8;
    for (a = 0; a < 8; a++, b--)
        ref[refLen + 1 + a] = (refCrc >> (b - 1)) & 1;
    ref[0] += 8;
}

// Message::periodic(): drive + functions F0..F15
static void refPeriodic(uint8_t address, uint8_t dir, uint8_t speed, const uint8_t funct[16])
{
    memset(ref, 0, sizeof(ref));
    refLen = 0;
    refAddr(address);
    ref[++refLen] = 0;
    ref[++refLen] = 0;
    ref[++refLen] = 1;
    ref[++refLen] = dir ? 1 : 0;
    for (uint8_t a = 0, b = 6; a < 7; a++, b--)
        ref[++refLen] = (speed & (1 << b)) >> b;
    ref[++refLen] = 0;
    ref[++refLen] = 1;
    ref[++refLen] = 1;
    ref[++refLen] = 1;
    for (uint8_t a = 0, b = 15; a < 16; a++, b--)
        ref[++refLen] = funct[b];
    ref[0] = refLen;
    refCRC();
}

// Message::centrale(): command station UID + counter
static void refCentrale(uint32_t uid, uint16_t counter)
{
    memset(ref, 0, sizeof(ref));
    refLen = 0;
    refAddr0();
    for (uint8_t a = 0, b = 5; a < 6; a++, b--)
        ref[++refLen] = (0x3D & (1 << b)) >> b;
    for (uint8_t a = 0, b = 31; a < 32; a++, b--)
        ref[++refLen] = (uid >> b) & 1UL;
    for (uint8_t a = 0, b = 15; a < 16; a++, b--)
        ref[++refLen] = (counter >> b) & 1U;
    ref[0] = refLen;
    refCRC();
}

// Message::setSID(): assignment of a rail address
static void refSetSID(uint8_t address, const uint8_t uid[4])
{
    memset(ref, 0, sizeof(ref));
    refLen = 0;
    refAddr0();
    for (uint8_t a = 0, b = 5; a < 6; a++, b--)
        ref[++refLen] = (0x3B & (1 << b)) >> b;
    for (uint8_t a = 0; a < 7; a++)
        ref[++refLen] = 0;
    for (uint8_t a = 0, b = 6; a < 7; a++, b--)
        ref[++refLen] = (address & (1 << b)) >> b;
    for (uint8_t j = 0; j < 4; j++)
        for (uint8_t a = 0, b = 7; a < 8; a++, b--)
            ref[++refLen] = (uid[j] & (1 << b)) >> b;
    ref[0] = refLen;
    refCRC();
}

static void assertSameAsReference(const MfxFrame &f)
{
    uint8_t buff[100];
    TEST_ASSERT_FALSE(f.overflow());
    TEST_ASSERT_TRUE(f.toBuffer(buff, sizeof(buff)));
    TEST_ASSERT_EQUAL_UINT8(ref[0], buff[0]);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(ref, buff, ref[0] + 1);
}

// ---------------------------------------------------------------------------
//  Tests
// ---------------------------------------------------------------------------

void setUp() {}
void tearDown() {}

// Frame seen on an MS2 (published on the ATC page): HQ-UID 0x4746602A,
// counter 6, CRC 0xD9.
void test_crc_of_ms2_broadcast()
{
    MfxFrame f;
    f.broadcast();
    f.centrale(0x4746602AUL, 6);
    TEST_ASSERT_EQUAL_UINT8(63, f.size());
    TEST_ASSERT_EQUAL_HEX8(0xD9, MfxFrame::crc(f, f.size()));
    f.finish();
    TEST_ASSERT_EQUAL_UINT8(71, f.size());
}

void test_periodic_frames_identical()
{
    const uint8_t sids[] = {1, 2, 4, 5, 63, 127};
    const uint8_t speeds[] = {0, 1, 12, 64, 127};
    for (uint8_t s : sids)
        for (uint8_t sp : speeds)
            for (uint8_t dir = 0; dir < 2; dir++)
            {
                uint8_t funct[16] = {0};
                uint16_t mask = 0;
                // a different pattern of functions for each case
                for (uint8_t n = 0; n < 16; n++)
                    if (((s + sp + dir + n) % 3) == 0)
                    {
                        funct[n] = 1;
                        mask |= (uint16_t)(1u << n);
                    }
                refPeriodic(s, dir, sp, funct);

                MfxFrame f;
                f.address(s);
                f.drive(dir != 0, sp);
                f.functions16(mask);
                f.finish();
                assertSameAsReference(f);
            }
}

void test_centrale_frame_identical()
{
    const uint16_t counters[] = {0, 1, 5, 255};
    for (uint16_t c : counters)
    {
        refCentrale(0x476bb7dcUL, c);
        MfxFrame f;
        f.broadcast();
        f.centrale(0x476bb7dcUL, c);
        f.finish();
        assertSameAsReference(f);
    }
}

void test_bind_frame_identical()
{
    const uint8_t uid150X5[4] = {0xFF, 0xE9, 0xB1, 0x94};
    const uint8_t uid040[4] = {0xF9, 0xE6, 0x10, 0xE0};

    refSetSID(5, uid150X5);
    MfxFrame f;
    f.broadcast();
    f.bind(5, 0xFFE9B194UL);
    f.finish();
    assertSameAsReference(f);

    refSetSID(4, uid040);
    MfxFrame g;
    g.broadcast();
    g.bind(4, 0xF9E610E0UL);
    g.finish();
    assertSameAsReference(g);
}

// Address formats (Krauss 2.2.4)
void test_address_formats()
{
    MfxFrame f;
    f.address(127); // 10 + 7 bits
    TEST_ASSERT_EQUAL_UINT8(9, f.size());
    TEST_ASSERT_TRUE(f.bitAt(0));
    TEST_ASSERT_FALSE(f.bitAt(1));

    f.clear();
    f.address(128); // 110 + 9 bits
    TEST_ASSERT_EQUAL_UINT8(12, f.size());
    TEST_ASSERT_TRUE(f.bitAt(0));
    TEST_ASSERT_TRUE(f.bitAt(1));
    TEST_ASSERT_FALSE(f.bitAt(2));
    TEST_ASSERT_FALSE(f.bitAt(3)); // 128 on 9 bits = 0 1 0000000
    TEST_ASSERT_TRUE(f.bitAt(4));

    f.clear();
    f.address(2047); // 1110 + 11 bits
    TEST_ASSERT_EQUAL_UINT8(15, f.size());
    TEST_ASSERT_FALSE(f.bitAt(3));

    f.clear();
    f.address(0x3FFF); // 1111 + 14 bits
    TEST_ASSERT_EQUAL_UINT8(18, f.size());
    for (uint8_t i = 0; i < 18; i++)
        TEST_ASSERT_TRUE(f.bitAt(i));
}

// Decoder search (Krauss 3.2.3), as in the search log of the ATC page:
// 111010 CCCCCC UUUU...U after the broadcast address
void test_search_frame()
{
    MfxFrame f;
    f.broadcast();
    f.search(3, 0xDFFFFFFFUL);
    TEST_ASSERT_EQUAL_UINT8(9 + 6 + 6 + 32, f.size());

    const char *expected = "100000000"   // address 0, 7-bit format
                           "111010"      // search
                           "000011"      // count = 3
                           "11011111111111111111111111111111";
    for (uint8_t i = 0; i < f.size(); i++)
        TEST_ASSERT_EQUAL_MESSAGE(expected[i] == '1', f.bitAt(i), "bit of the search frame");

    // count is kept on 6 bits
    MfxFrame g;
    g.search(32, 0);
    TEST_ASSERT_EQUAL(true, g.bitAt(6));   // 32 = 100000
    TEST_ASSERT_EQUAL(false, g.bitAt(11));
}

/*
 Frames recorded on the track of a Marklin Gleisbox (MS2) reading the CVs
 of a decoder with the rail address 1. They include the CRC.
*/
static void checkReadCV(uint16_t cv, uint8_t index, uint8_t bytes, const char *recorded)
{
    MfxFrame f;
    f.address(1);
    f.readCV(cv, index, bytes);
    f.finish();
    TEST_ASSERT_EQUAL_UINT8(strlen(recorded), f.size());
    for (uint8_t i = 0; i < f.size(); i++)
        TEST_ASSERT_EQUAL_MESSAGE(recorded[i] == '1', f.bitAt(i), "bit of the read CV frame");
}

void test_read_cv_frames_as_recorded()
{
    checkReadCV(0x000, 0x00, 2, "10000000111100000000000000000000111100110");
    checkReadCV(0x080, 0x00, 2, "10000000111100000100000000000000101010000");
    checkReadCV(0x003, 0x01, 4, "10000000111100000000000110000011011001100");
    checkReadCV(0x02F, 0x01, 2, "10000000111100000001011110000010110010111");
}

// Example of "Die mfx-Rahmenstruktur" (PING): address 1, UID FFFEB2F9
void test_verify_frame()
{
    MfxFrame f;
    f.address(1);
    f.verify(0xFFFEB2F9UL);
    f.finish();
    const char *expected = "100000001" "111100" "11111111111111101011001011111001" "00001100";
    TEST_ASSERT_EQUAL_UINT8(strlen(expected), f.size());
    for (uint8_t i = 0; i < f.size(); i++)
        TEST_ASSERT_EQUAL_MESSAGE(expected[i] == '1', f.bitAt(i), "bit of the verify frame");
}

// Krauss 3.1.6: 100 NNNNNNN 0 F, one function (F16 and beyond)
void test_single_function_frame()
{
    MfxFrame f;
    f.address(5);
    f.function(20, true);
    f.finish();
    const char *expected = "100000101" "100" "0010100" "0" "1";
    TEST_ASSERT_EQUAL_UINT8(strlen(expected) + 8, f.size());
    for (uint8_t i = 0; i < strlen(expected); i++)
        TEST_ASSERT_EQUAL_MESSAGE(expected[i] == '1', f.bitAt(i), "bit of the function frame");
    // the CRC is the one of the bits before it
    uint8_t crc = 0;
    for (uint8_t i = 0; i < 8; i++)
        crc = (uint8_t)((crc << 1) | (f.bitAt(strlen(expected) + i) ? 1 : 0));
    TEST_ASSERT_EQUAL_HEX8(MfxFrame::crc(f, strlen(expected)), crc);

    MfxFrame g;
    g.address(5);
    g.function(127, false);
    g.finish();
    const char *expected2 = "100000101" "100" "1111111" "0" "0";
    for (uint8_t i = 0; i < strlen(expected2); i++)
        TEST_ASSERT_EQUAL_MESSAGE(expected2[i] == '1', g.bitAt(i), "bit of the function frame F127");
}

void test_count_codes()
{
    TEST_ASSERT_EQUAL_UINT8(0, MfxFrame::countCode(1));
    TEST_ASSERT_EQUAL_UINT8(1, MfxFrame::countCode(2));
    TEST_ASSERT_EQUAL_UINT8(2, MfxFrame::countCode(4));
    TEST_ASSERT_EQUAL_UINT8(3, MfxFrame::countCode(8));
    TEST_ASSERT_EQUAL_UINT8(0, MfxFrame::countCode(3));
    TEST_ASSERT_EQUAL_UINT8(8, MfxFrame::countBytes(3));
}

void test_buffer_too_small_and_overflow()
{
    MfxFrame f;
    f.broadcast();
    f.centrale(0x476bb7dcUL, 5);
    f.finish();
    uint8_t small[10];
    TEST_ASSERT_FALSE(f.toBuffer(small, sizeof(small)));

    MfxFrame big;
    for (uint8_t i = 0; i < 5; i++)
        big.centrale(0xFFFFFFFFUL, 0xFFFF); // 5 x 54 bits > 128
    TEST_ASSERT_TRUE(big.overflow());
    TEST_ASSERT_EQUAL_UINT8(MfxFrame::MAX_BITS, big.size());
}

// isBind recognises a bind (broadcast as the station sends it, or to an
// address of each length), and nothing else
void test_is_bind()
{
    uint8_t b[MfxFrame::MAX_BITS + 1];
    const uint16_t sids[4] = {0, 100, 300, 1500};
    for (uint8_t i = 0; i < 4; i++)
    {
        MfxFrame f;
        if (sids[i] == 0)
            f.broadcast();
        else
            f.address(sids[i]);
        f.bind(4, 0xF9E610E0UL);
        f.finish();
        TEST_ASSERT_TRUE(f.toBuffer(b, sizeof(b)));
        TEST_ASSERT_TRUE(MfxFrame::isBind(b));
    }
    MfxFrame s;
    s.broadcast();
    s.search(0, 0xFFFFFFFFUL);
    s.finish();
    s.toBuffer(b, sizeof(b));
    TEST_ASSERT_FALSE(MfxFrame::isBind(b));
    MfxFrame c;
    c.broadcast();
    c.centrale(0x476BB7DC, 30);
    c.finish();
    c.toBuffer(b, sizeof(b));
    TEST_ASSERT_FALSE(MfxFrame::isBind(b));
    MfxFrame v;
    v.address(4);
    v.verify(0xF9E610E0UL);
    v.finish();
    v.toBuffer(b, sizeof(b));
    TEST_ASSERT_FALSE(MfxFrame::isBind(b));
    b[0] = 3;
    b[1] = 1; b[2] = 0; b[3] = 0;
    TEST_ASSERT_FALSE(MfxFrame::isBind(b)); // too short
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_crc_of_ms2_broadcast);
    RUN_TEST(test_periodic_frames_identical);
    RUN_TEST(test_centrale_frame_identical);
    RUN_TEST(test_bind_frame_identical);
    RUN_TEST(test_is_bind);
    RUN_TEST(test_address_formats);
    RUN_TEST(test_search_frame);
    RUN_TEST(test_read_cv_frames_as_recorded);
    RUN_TEST(test_verify_frame);
    RUN_TEST(test_single_function_frame);
    RUN_TEST(test_count_codes);
    RUN_TEST(test_buffer_too_small_and_overflow);
    return UNITY_END();
}

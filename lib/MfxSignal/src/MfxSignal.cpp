/*
  MfxSignal.cpp
*/

#include "MfxSignal.h"

bool MfxSignal::windowLevel = true;
uint16_t MfxSignal::answerFlipUs = MfxSignal::FORCE_US;
bool MfxSignal::answerPulse = true;
uint16_t MfxSignal::pulseUs = 10;
volatile uint8_t MfxSignal::lastEnd = MfxSignal::END_NONE;

void MfxSignal::push(bool level0, bool level1, uint16_t duration0, uint16_t duration1)
{
    if (mCount >= MAX_ITEMS)
    {
        mOverflow = true;
        return;
    }
    mItems[mCount].level0 = level0;
    mItems[mCount].level1 = level1;
    mItems[mCount].duration0 = duration0;
    mItems[mCount].duration1 = duration1;
    mCount++;
}

void MfxSignal::hold(uint32_t us)
{
    if (us > 65534)
        us = 65534;
    const uint16_t first = (uint16_t)(us / 2);
    push(mLevel, mLevel, first, (uint16_t)(us - first));
}

void MfxSignal::half(uint16_t us)
{
    if (us > 65534)
        us = 65534;
    mLevel = !mLevel;
    const uint16_t first = (uint16_t)(us / 2);
    push(mLevel, mLevel, first, (uint16_t)(us - first));
}

uint32_t MfxSignal::durationUs() const
{
    uint32_t us = 0;
    for (uint16_t i = 0; i < mCount; i++)
        us += (uint32_t)mItems[i].duration0 + mItems[i].duration1;
    return us;
}

void MfxSignal::request(const uint8_t *insert)
{
    if (insert)
    {
        sync();
        frame(insert);
    }
    for (uint8_t i = 0; i < FEEDBACK_SYNCS; i++)
        sync();
    bit0();
    bit0();
    bit1();
    bit1();
    if (answerPulse && mLevel == windowLevel)
    {
        // Already at the window level: a short pulse at the other level all
        // the same, so that a transition follows the bits 0011 (Gleisbox)
        push(mLevel, !mLevel, FORCE_US, pulseUs);
        lastEnd = END_PULSE;
    }
    else if (answerFlipUs != 0 && mLevel != windowLevel)
    {
        forceLevel(windowLevel, answerFlipUs);
        lastEnd = END_FLIP;
    }
    else
        lastEnd = END_NONE;
}

void MfxSignal::forceLevel(bool level, uint16_t delayUs)
{
    if (mLevel == level)
        return;
    push(mLevel, level, delayUs, FORCE_US);
    mLevel = level;
}

void MfxSignal::feedbackWindows(uint32_t *windowUs, uint32_t *referenceUs, const uint8_t *insert)
{
    request(insert);

    if (windowUs)
        *windowUs = durationUs();
    const bool answerLevel = mLevel; // windowLevel, unless F0
    hold(FEEDBACK_WINDOW_US);

    sync();
    forceLevel(!answerLevel);

    if (referenceUs)
        *referenceUs = durationUs();
    hold(FEEDBACK_WINDOW_US);

    for (uint8_t i = 0; i < FEEDBACK_END_SYNCS; i++)
        sync();
    hold(END_HOLD_US);
}

void MfxSignal::dataWindow(uint8_t bytes, uint32_t *pauseUs, const uint8_t *insert)
{
    if (bytes > DATA_MAX_BYTES)
        bytes = DATA_MAX_BYTES;
    request(insert);

    if (pauseUs)
        *pauseUs = durationUs();
    // One item per period: the pause level, then the pulse. The next item
    // starts again at the pause level (end of the pulse).
    const bool l = mLevel;
    for (uint8_t i = 0; i < DATA_LEAD_CLOCKS; i++)
        push(l, !l, DATA_CLOCK_US - DATA_PULSE_US, DATA_PULSE_US);
    const uint16_t half = DATA_CLOCK_US / 2;
    const uint16_t pulses = 2 * dataClocks(bytes);
    for (uint16_t i = 0; i < pulses; i++)
        push(l, !l, half - DATA_PULSE_US, DATA_PULSE_US);
    push(l, l, HALF_US / 2, HALF_US / 2); // back to the pause level
    mLevel = l;

    for (uint8_t i = 0; i < DATA_END_SYNCS; i++)
        sync();
    hold(END_HOLD_US);
}

void MfxSignal::bit0()
{
    mLevel = !mLevel; // transition at the start
    push(mLevel, mLevel);
}

void MfxSignal::bit1()
{
    const bool first = !mLevel; // transition at the start
    mLevel = !first;            // and in the middle
    push(first, mLevel);
}

/*
 Same pattern as setMFXsync() of version 0.10.0 (from the original DirectMFX).
 Starting from level L:
   item 1 : !L  !L   (transition, 100 µs)
   item 2 :  L  !L   (transition, then transition in the middle)
   item 3 : !L   L   (no transition at the start)
   item 4 :  L  !L   (no transition at the start)
   item 5 :  L   L   (transition, 100 µs)
 It ends at L.
*/
void MfxSignal::sync()
{
    const bool l = mLevel;
    push(!l, !l);
    push(l, !l);
    push(!l, l);
    push(l, !l);
    push(l, l);
    mLevel = l;
}

void MfxSignal::frame(const uint8_t *buff)
{
    const uint8_t nbits = buff[0];
    uint8_t ones = 0;
    for (uint8_t i = 1; i <= nbits; i++)
    {
        if (buff[i])
        {
            bit1();
            if (++ones == 8)
            {
                // Bit stuffing: a 0 after 8 consecutive 1 bits.
                // It does not consume a bit of the frame.
                bit0();
                ones = 0;
            }
        }
        else
        {
            bit0();
            ones = 0;
        }
    }
}

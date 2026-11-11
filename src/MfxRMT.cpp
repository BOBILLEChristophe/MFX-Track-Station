
/*

   MfxRMT.cpp

   The track signal must never stop: a decoder expects a transition at least
   every 100 µs (except inside a synchronisation pattern).

   - signalTask() builds the next chunk while the current one is being sent:
     rmt_transmit() waits for the end of the previous chunk (a queue of one
     transaction), then starts the new one at once.
   - RMT driver of ESP-IDF 5 (driver/rmt_tx.h, Arduino core 3.x), which
     replaces the legacy one (driver/rmt.h) used up to version 0.12.0.
   - A chunk is either a frame between two synchronisations, or a single
     synchronisation when there is nothing to send.
   - Between two chunks the RMT is briefly idle (a few tens of µs). Each
     chunk ends at its own last level (eot_level of the transmission), so
     the line keeps its level: no spurious transition, only the last
     half-bit of a synchronisation is a little longer.
   - A feedback request (Feedback.h) is sent before the other frames. The
     time at which its chunk starts is handed to Feedback, which counts the
     decoder answer in the windows (1-bit feedback) or times the edges of
     the answer during the pause (data feedback, reading CVs).
   - After a feedback request and after a bind, the line is held still for
     1.6 ms, as the Gleisbox does (MfxSignal::END_HOLD_US). A bind is first
     followed by two synchronisations, the second one merging with the hold:
     the first one must stay complete, it ends the frame. The idle time
     between two chunks then falls in that hold.
   - As a Marklin MS2 does, the command station frame (UID and counter) is
     inserted between the question and the feedback request (MfxSignal).
   - DCC (DccLocos.h), as the Gleisbox: a round of DCC packets, one per
     locomotive, one chunk per packet, then about 30 ms of mfx
     (DccLocos::MFX_SLOT_US). An order of the master goes into the next
     chunk. Between two DCC chunks, the idle time of the RMT falls in the
     first half of a start bit, which may be stretched (DccPacket.h).

*/

#include "MfxRMT.h"
#include "Feedback.h"
#include "Centrale.h"
#include "MfxFrame.h"
#include "DccLocos.h"
#include "DccPacket.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h" // esp_rom_gpio_pad_select_gpio, esp_rom_gpio_connect_out_signal
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "soc/gpio_sig_map.h" // RMT_SIG_OUT0_IDX
#include "soc/soc_caps.h"     // SOC_RMT_MEM_WORDS_PER_CHANNEL

#define RMT_RESOLUTION_HZ 1000000 // 1 µs per tick

rmt_symbol_word_t RMTChannel::sItems[2][MfxSignal::MAX_ITEMS];
uint8_t RMTChannel::sCurrent = 0;

static rmt_channel_handle_t sChannel = NULL;
static rmt_encoder_handle_t sEncoder = NULL; // copies the items as they are

static void fatal(const char *what, esp_err_t err)
{
    Serial.printf("MfxRMT: %s failed (%s), no track signal\n", what, esp_err_to_name(err));
    while (1)
        vTaskDelay(pdMS_TO_TICKS(1000));
}

void RMTChannel::setup(gpio_num_t IN1_pin, gpio_num_t IN2_pin)
{
    rmt_tx_channel_config_t config = {};
    config.gpio_num = IN1_pin;
    config.clk_src = RMT_CLK_SRC_DEFAULT; // APB, 80 MHz
    config.resolution_hz = RMT_RESOLUTION_HZ;
    // The 8 blocks of 64 items (the whole RMT memory, no other channel is
    // used): the longest chunk (MfxSignal::MAX_ITEMS = 434, a frame with
    // the command station frame and a data feedback of 8 bytes) fits in the
    // RMT memory (512 items), with room for the end marker: no refill by
    // interrupt during the transmission. The channel is then channel 0.
    config.mem_block_symbols = 8 * SOC_RMT_MEM_WORDS_PER_CHANNEL;
    // One transaction at a time: rmt_transmit() returns when the chunk has
    // started, the next call waits for its end (as rmt_write_items did)
    config.trans_queue_depth = 1;
    config.flags.init_level = 0;
    esp_err_t err = rmt_new_tx_channel(&config, &sChannel);
    if (err != ESP_OK)
        fatal("rmt_new_tx_channel", err);
    rmt_copy_encoder_config_t copy = {};
    err = rmt_new_copy_encoder(&copy, &sEncoder);
    if (err != ESP_OK)
        fatal("rmt_new_copy_encoder", err);
    err = rmt_enable(sChannel);
    if (err != ESP_OK)
        fatal("rmt_enable", err);

    // Inverted signal: IN2 gets the output of the RMT channel 0, inverted
    // by the GPIO matrix (the whole memory is taken: it can only be channel 0)
    gpio_num_t gpioNum = (gpio_num_t)(IN2_pin);
    esp_rom_gpio_pad_select_gpio(gpioNum);
    gpio_set_direction(gpioNum, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(gpioNum, RMT_SIG_OUT0_IDX, true, false);

    // The queue is normally created in setup(); create it here only if it is missing
    if (mfxQueue == NULL)
        mfxQueue = xQueueCreate(10, BUFFER_SIZE);
    if (mfxQueue == NULL)
    {
        Serial.println("Error: Unable to create queue");
        while (1)
            ;
    }
    xTaskCreatePinnedToCore(signalTask, "Signal Task", 4 * 1024, NULL, 5, NULL, 1);
}

/*
 Copies the chunk into the free buffer and starts it as soon as the previous
 chunk has been sent. Returns the time at which it started (esp_timer, µs).
*/
int64_t RMTChannel::send(const MfxSignal &signal)
{
    rmt_symbol_word_t *items = sItems[sCurrent];
    const uint16_t n = signal.count();
    for (uint16_t i = 0; i < n; i++)
    {
        const MfxItem &it = signal.item(i);
        items[i].level0 = it.level0;
        items[i].duration0 = it.duration0;
        items[i].level1 = it.level1;
        items[i].duration1 = it.duration1;
    }

    // If the next chunk is a little late, the line keeps the level this
    // chunk ends with
    rmt_transmit_config_t tx = {};
    tx.loop_count = 0;
    tx.flags.eot_level = signal.level() ? 1 : 0;
    // Waits for the end of the previous chunk, then starts this one
    rmt_transmit(sChannel, sEncoder, items, n * sizeof(rmt_symbol_word_t), &tx);
    const int64_t start = esp_timer_get_time();

    sCurrent ^= 1; // the other buffer is free again only after this chunk ends
    return start;
}

volatile bool RMTChannel::framePause = true;

void RMTChannel::signalTask(void *parameter)
{
    Serial.printf("Signal task running on core %d\n", xPortGetCoreID());

    static MfxSignal signal; // static: too large for the task stack
    uint8_t frame[BUFFER_SIZE];
    bool lastWasFeedback = false;
    static Dcc::Packet dcc[DccLocos::MAX_ROUND];
    int64_t dccEnd = 0; // end of the last DCC round (esp_timer, µs)

    for (;;)
    {
        // DCC round: when an order waits, or after the mfx slot. The
        // previous chunk has just started: the time is about its start.
        if (DccLocos::enabled() &&
            (DccLocos::urgent() ||
             (DccLocos::count() != 0 && esp_timer_get_time() - dccEnd >= (int64_t)DccLocos::MFX_SLOT_US)))
        {
            const uint8_t n = DccLocos::round(dcc, DccLocos::MAX_ROUND);
            for (uint8_t i = 0; i < n; i++)
            {
                signal.clear();
                Dcc::encode(signal, dcc[i], i == 0, i + 1 < n);
                const int64_t start = send(signal);
                dccEnd = start + signal.durationUs();
            }
            if (n)
                continue;
        }

        signal.clear(); // keeps the level reached by the previous chunk
        const bool startLevel = signal.level();
        bool feedback = false;
        uint8_t dataBytes = 0; // 0: 1-bit feedback
        uint32_t windowUs = 0, referenceUs = 0, pauseUs = 0;

        // After a feedback request, one chunk from mfxQueue first: during a
        // decoder search, the locomotives keep their refresh
        if (!lastWasFeedback && Feedback::takeRequest(frame, sizeof(frame), &dataBytes))
        {
            // The command station frame, sent between the question and the
            // request, as the MS2 does (some decoders answer only then)
            MfxFrame station;
            station.broadcast();
            station.centrale(Centrale::gUid(), Centrale::counter());
            station.finish();
            uint8_t stationBuff[BUFFER_SIZE];
            const uint8_t *insert = station.toBuffer(stationBuff, sizeof(stationBuff)) ? stationBuff : nullptr;

            signal.sync();
            signal.frame(frame);
            if (dataBytes)
                signal.dataWindow(dataBytes, &pauseUs, insert); // read CV: pause with clock pulses
            else
                signal.feedbackWindows(&windowUs, &referenceUs, insert); // 1 bit: two windows
            feedback = true;
        }
        else if (xQueueReceive(mfxQueue, frame, 0) == pdPASS)
        {
            // Krauss 2.2.1: a frame between two synchronisations
            signal.sync();
            signal.frame(frame);
            signal.sync();
            if (MfxFrame::isBind(frame))
            {
                // As the Gleisbox: a second synchronisation, whose last
                // 100 µs half-bit merges with the hold. Holding right after
                // the first one swallowed its last edge: the bind was never
                // ended for the decoders (nor for the sniffer)
                signal.sync();
                signal.hold(MfxSignal::END_HOLD_US);
            }
            else if (framePause)
            {
                signal.sync();
                signal.hold(MfxSignal::END_HOLD_US);
            }
        }
        else
        {
            // Nothing to send: keep the line alive
            signal.sync();
        }

        if (signal.overflow())
        {
            Serial.println("MfxRMT: frame too long, not sent");
            if (feedback)
                Feedback::cancel();
            feedback = false;
            signal.clear();
            signal.setLevel(startLevel); // the line is still at this level
            signal.sync();
        }

        const int64_t start = send(signal);
        if (feedback)
        {
            if (dataBytes)
                Feedback::dataStarted(start, pauseUs, dataBytes);
            else
                Feedback::windowsStarted(start, windowUs, referenceUs);
        }
        lastWasFeedback = feedback;
    }
}

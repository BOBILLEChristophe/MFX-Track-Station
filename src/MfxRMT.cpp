
/*

   MfxRMT.cpp

   The track signal must never stop: a decoder expects a transition at least
   every 100 µs (except inside a synchronisation pattern).

   - signalTask() builds the next chunk while the current one is being sent:
     rmt_write_items(..., false) waits for the end of the previous chunk, then
     starts the new one at once.
   - A chunk is either a frame between two synchronisations, or a single
     synchronisation when there is nothing to send.
   - Between two chunks the RMT is briefly idle (a few tens of µs). Its idle
     level is set to the level of the end of the chunk being sent, so the
     line keeps its level: no spurious transition, only the last half-bit
     of a synchronisation is a little longer.
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

*/

#include "MfxRMT.h"
#include "Feedback.h"
#include "Centrale.h"
#include "MfxFrame.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h" // esp_rom_gpio_pad_select_gpio, esp_rom_gpio_connect_out_signal

#define RMT_CLOCK_DIVIDER 80 // 80 MHz / 80 = 1 µs per tick

rmt_item32_t RMTChannel::sItems[2][MfxSignal::MAX_ITEMS];
uint8_t RMTChannel::sCurrent = 0;

void RMTChannel::setup(gpio_num_t IN1_pin, gpio_num_t IN2_pin)
{
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNEL_0;
    config.clk_div = RMT_CLOCK_DIVIDER;
    config.gpio_num = (gpio_num_t)IN1_pin;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    // The 8 blocks of 64 items (the whole RMT memory, no other channel is
    // used): the longest chunk (MfxSignal::MAX_ITEMS = 434, a frame with
    // the command station frame and a data feedback of 8 bytes) fits in the
    // RMT memory (512 items), with room for the end marker: no refill by
    // interrupt during the transmission.
    config.mem_block_num = 8;

    // Signal inversé : IN2 reçoit la sortie du RMT, inversée par la matrice GPIO.
    // esp_rom_gpio_* existe avec le core Arduino 2.x (ESP-IDF 4.4) comme 3.x (ESP-IDF 5),
    // contrairement à GPIO_PIN_MUX_REG et gpio_matrix_out, retirés du core 3.x.
    gpio_num_t gpioNum = (gpio_num_t)(IN2_pin);
    esp_rom_gpio_pad_select_gpio(gpioNum);
    gpio_set_direction(gpioNum, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(gpioNum, RMT_SIG_OUT0_IDX + RMT_CHANNEL_0, true, false);
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, ESP_INTR_FLAG_SHARED);

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
    rmt_item32_t *items = sItems[sCurrent];
    const uint16_t n = signal.count();
    for (uint16_t i = 0; i < n; i++)
    {
        const MfxItem &it = signal.item(i);
        items[i].level0 = it.level0;
        items[i].duration0 = it.duration0;
        items[i].level1 = it.level1;
        items[i].duration1 = it.duration1;
    }

    // Waits for the end of the previous chunk, then starts this one
    rmt_write_items(RMT_CHANNEL_0, items, n, false);
    const int64_t start = esp_timer_get_time();

    // While this chunk is being sent: if the next one is a little late, the
    // line keeps the level this chunk ends with.
    rmt_set_idle_level(RMT_CHANNEL_0, true, signal.level() ? RMT_IDLE_LEVEL_HIGH : RMT_IDLE_LEVEL_LOW);

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

    for (;;)
    {
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

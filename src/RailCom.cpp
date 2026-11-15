/*

   RailCom.cpp

   See RailCom.h.

*/

#include "RailCom.h"
#include "DccPacket.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "hal/gpio_ll.h"   // inline register access, usable in the interrupts
#include "soc/gpio_struct.h"
#include "esp_private/cache_utils.h" // spi_flash_cache_enabled

namespace RailCom
{
    volatile uint16_t latencyUs = 3;

    // Window before the expected edge, and after it (the edge must fall in it)
    static const uint16_t WINDOW_BEFORE_US = 40;
    static const uint16_t WINDOW_AFTER_US = 40;

    static gpio_num_t sGate = GPIO_NUM_NC;
    static gpio_num_t sIn1 = GPIO_NUM_NC;
    static bool sEnabled = false;
    static gptimer_handle_t sTimer = NULL;

    static volatile bool sPending = false;   // a cutout waits for its edge
    static volatile int64_t sOpenUs = 0;     // window: from
    static volatile int64_t sCloseUs = 0;    // to
    static volatile bool sLevel = false;     // level of IN1 after the edge
    static volatile uint64_t sEdgeCount = 0; // gptimer count at the edge
    static volatile uint8_t sPhase = 0;      // 0: gate to 0 next, 1: gate to 1 next
    static volatile uint32_t sDone = 0, sSkipped = 0;

    static void IRAM_ATTR skip()
    {
        sPending = false;
        sSkipped = sSkipped + 1;
        gpio_ll_intr_disable(&GPIO, sIn1);
    }

    // Edge on IN1: the end of the end bit, if it is in the window and at the right level
    static void IRAM_ATTR edgeIsr(void *)
    {
        if (!sPending)
        {
            gpio_ll_intr_disable(&GPIO, sIn1);
            return;
        }
        const int64_t now = esp_timer_get_time();
        if (now < sOpenUs)
            return; // an edge of the packet, before the window
        if (now > sCloseUs || (gpio_ll_get_level(&GPIO, sIn1) != 0) != sLevel)
        {
            skip(); // window missed (the task was late): no cutout for this packet
            return;
        }
        // The GPIO interrupts run during the writes in flash too (NVS): the
        // timer functions are in flash, no cutout then
        if (!spi_flash_cache_enabled())
        {
            skip();
            return;
        }
        sPending = false;
        gpio_ll_intr_disable(&GPIO, sIn1);
        uint64_t count = 0;
        gptimer_get_raw_count(sTimer, &count);
        sEdgeCount = count;
        sPhase = 0;
        gptimer_alarm_config_t alarm = {};
        alarm.alarm_count = count + Dcc::RAILCOM_START_US - latencyUs;
        gptimer_set_alarm_action(sTimer, &alarm);
    }

    // Timer: gate to 0 (start of the cutout), then to 1 (end)
    static bool IRAM_ATTR alarmIsr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *, void *)
    {
        if (sPhase == 0)
        {
            gpio_ll_set_level(&GPIO, sGate, 0);
            sPhase = 1;
            gptimer_alarm_config_t alarm = {};
            alarm.alarm_count = sEdgeCount + Dcc::RAILCOM_END_US - latencyUs;
            gptimer_set_alarm_action(timer, &alarm);
        }
        else
        {
            gpio_ll_set_level(&GPIO, sGate, 1);
            sDone = sDone + 1;
            gptimer_set_alarm_action(timer, NULL); // no more alarm
        }
        return false;
    }

    bool setup(gpio_num_t gate, gpio_num_t in1)
    {
        if (gate == GPIO_NUM_NC)
            return false;
        sGate = gate;
        sIn1 = in1;

        // Gate at 1: AND gates transparent. gpio_config also selects the
        // GPIO function of the pad (GPIO 14 is a JTAG pin at reset)
        gpio_set_level(sGate, 1);
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << sGate;
        io.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io);
        gpio_set_level(sGate, 1);

        // Free-running timer, 1 µs
        gptimer_config_t tc = {};
        tc.clk_src = GPTIMER_CLK_SRC_DEFAULT;
        tc.direction = GPTIMER_COUNT_UP;
        tc.resolution_hz = 1000000;
        if (gptimer_new_timer(&tc, &sTimer) != ESP_OK)
        {
            Serial.println("RailCom: no timer, no cutout");
            sGate = GPIO_NUM_NC;
            return false;
        }
        gptimer_event_callbacks_t cbs = {};
        cbs.on_alarm = alarmIsr;
        gptimer_register_event_callbacks(sTimer, &cbs, NULL);
        gptimer_enable(sTimer);
        gptimer_start(sTimer);

        // IN1 read back on its pad (the RMT still drives it), interrupt on
        // both edges, off until a cutout is armed
        gpio_ll_input_enable(&GPIO, sIn1);
        gpio_set_intr_type(sIn1, GPIO_INTR_ANYEDGE);
        gpio_intr_disable(sIn1);
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) // already installed: fine
        {
            Serial.println("RailCom: no GPIO interrupt, no cutout");
            sGate = GPIO_NUM_NC;
            return false;
        }
        gpio_isr_handler_add(sIn1, edgeIsr, NULL);
        Serial.printf("RailCom: cutout on GPIO %d (%s)\n", (int)sGate, sEnabled ? "on" : "off");
        return true;
    }

    bool available()
    {
        return sGate != GPIO_NUM_NC;
    }

    bool enabled()
    {
        return sEnabled && available();
    }

    void setEnabled(bool on)
    {
        sEnabled = on && available();
        if (!sEnabled && available())
            gpio_set_level(sGate, 1);
    }

    void arm(int64_t startUs, uint32_t cutoutUs, bool levelAfter)
    {
        if (!enabled() || cutoutUs == 0)
            return;
        const int64_t edge = startUs + cutoutUs;
        const int64_t now = esp_timer_get_time();
        if (sPending)
        {
            if (now <= sCloseUs)
            {
                sSkipped = sSkipped + 1; // the previous one is still waiting for its edge
                return;
            }
            sPending = false; // its window is over without an edge: forgotten
            sSkipped = sSkipped + 1;
        }
        if (now > edge - WINDOW_BEFORE_US / 2)
        {
            sSkipped = sSkipped + 1; // too late already
            return;
        }
        sLevel = levelAfter;
        sOpenUs = edge - WINDOW_BEFORE_US;
        sCloseUs = edge + WINDOW_AFTER_US;
        sPending = true;
        gpio_intr_enable(sIn1);
    }

    uint32_t done()
    {
        return sDone;
    }

    uint32_t skipped()
    {
        return sSkipped;
    }
}

/*

  Centrale.cpp


*/

#include "Centrale.h"
#include <Preferences.h>
#include "rom/ets_sys.h" // ets_delay_us

uint32_t Centrale::m_uid = 0;
uint16_t Centrale::m_counter = Centrale::DEFAULT_MFX_COUNTER;
gpio_num_t Centrale::EN_pin = GPIO_NUM_0;
gpio_num_t Centrale::RST_pin = GPIO_NUM_NC;
gpio_num_t Centrale::FAULT_pin = GPIO_NUM_NC;
volatile bool Centrale::m_power = false;
volatile uint32_t Centrale::m_powerOnMs = 0;
volatile bool Centrale::m_overload = false;
portMUX_TYPE Centrale::m_mux = portMUX_INITIALIZER_UNLOCKED;

static const char *NVS_NAMESPACE = "centrale";
static const char *NVS_COUNTER_KEY = "mfxCounter";

uint32_t Centrale::gUid()
{
    return m_uid;
}

uint16_t Centrale::counter()
{
    return m_counter;
}

void Centrale::setCounter(uint16_t value)
{
    m_counter = value;
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false))
    {
        prefs.putUShort(NVS_COUNTER_KEY, value);
        prefs.end();
    }
}

void Centrale::setup(uint32_t uid, gpio_num_t EN)
{
    m_uid = uid;
    EN_pin = EN;

    // Read-write: a read-only open fails (and logs an error) while the
    // namespace does not exist yet, i.e. on the first boot.
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false))
    {
        m_counter = prefs.getUShort(NVS_COUNTER_KEY, DEFAULT_MFX_COUNTER);
        prefs.end();
    }

    // gpio_config: also selects the GPIO function of the pad (GPIO 13 of the
    // board proto 2 is a JTAG pin at reset)
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << EN_pin;
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level(EN_pin, LOW);
}

/*
 Appelée par Rocrail (Stop / Go). Un Go est le seul moyen de sortir d'une surcharge.
*/
void Centrale::setPower(bool power)
{
    const bool wasOn = m_power;
    portENTER_CRITICAL(&m_mux);
    m_power = power;
    if (power)
        m_overload = false;
    if (power && !wasOn)
        m_powerOnMs = millis();
    if (power && RST_pin != GPIO_NUM_NC)
    {
        // Latch of the hardware protection cleared (pulse on CLR), then armed
        gpio_set_level(RST_pin, LOW);
        ets_delay_us(2);
        gpio_set_level(RST_pin, HIGH);
    }
    gpio_set_level(EN_pin, power ? HIGH : LOW);
    portEXIT_CRITICAL(&m_mux);
    Serial.printf("power %s\n", power ? "on" : "off");
}

bool Centrale::power()
{
    return m_power;
}

uint32_t Centrale::poweredForMs()
{
    if (!m_power)
        return 0;
    const uint32_t d = millis() - m_powerOnMs;
    return d ? d : 1;
}

/*
 Appelée par la surveillance du courant. La voie reste coupée jusqu'au prochain Go.
*/
void Centrale::setOverload()
{
    portENTER_CRITICAL(&m_mux);
    gpio_set_level(EN_pin, LOW);
    m_power = false;
    m_overload = true;
    portEXIT_CRITICAL(&m_mux);
}

bool Centrale::overload()
{
    return m_overload;
}

void Centrale::setupProtection(gpio_num_t rst, gpio_num_t fault)
{
    RST_pin = rst;
    FAULT_pin = fault;
    if (RST_pin != GPIO_NUM_NC)
    {
        // Latch held cleared until the first Go (R46 pulls it low as well)
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << RST_pin;
        io.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io);
        gpio_set_level(RST_pin, LOW);
    }
    if (FAULT_pin != GPIO_NUM_NC)
    {
        // GPIO 39: input only, no pull-up, driven by the 74HC74
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << FAULT_pin;
        io.mode = GPIO_MODE_INPUT;
        gpio_config(&io);
    }
}

bool Centrale::hasProtection()
{
    return RST_pin != GPIO_NUM_NC && FAULT_pin != GPIO_NUM_NC;
}

bool Centrale::protectionTripped()
{
    return FAULT_pin != GPIO_NUM_NC && gpio_get_level(FAULT_pin) == 1;
}

/*

   Board.h

   Pins of the board, chosen at build time:
     - default (environment esp32dev): the current prototype;
     - BOARD_PROTO2 (environment proto2, platformio.ini): the KiCad board
       "MFX Track Station proto 2" (Christophe Bobille, Thierry Dussart,
       November 2026), with:
         . the H-bridge commands through SN74HCT08N AND gates, whose second
           input is CUTOUT_GATE (GPIO 14): RailCom cutout (RailCom.h);
         . the hardware overcurrent protection (LM393 + 74HC74 latch):
           TRK_RST (GPIO 23) clears / arms the latch, FAULT (GPIO 39)
           reads it (Centrale.h);
         . the shunt R6 // R50 = 0.25 ohm (current measurement, CurrentMonitor.h).

   GPIO_NUM_NC: not on this board (the matching function is off).

*/

#pragma once

#include "driver/gpio.h"

#if defined(BOARD_PROTO2)

#define BOARD_NAME "proto 2 (KiCad)"
static const gpio_num_t IN1_PIN = GPIO_NUM_19;             // TRK_IN1 -> U11 (AND) -> IN1 of the L6203
static const gpio_num_t IN2_PIN = GPIO_NUM_18;             // TRK_IN2 -> U11 (AND) -> IN2 of the L6203
static const gpio_num_t EN_PIN = GPIO_NUM_13;              // TRK_EN -> R43 -> ENABLE of the L6203
static const gpio_num_t CURRENT_PIN = GPIO_NUM_36;         // I_ADC (R7 + C46)
static const gpio_num_t FEEDBACK_PIN_BOARD = GPIO_NUM_32;  // FB_OUT, comparator MCP6541
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_4;
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_5;
static const gpio_num_t CUTOUT_PIN = GPIO_NUM_14;          // CUTOUT_GATE: 0 = both half-bridges low (RailCom)
static const gpio_num_t TRK_RST_PIN = GPIO_NUM_23;         // CLR of the 74HC74: 0 = latch cleared, 1 = armed
static const gpio_num_t FAULT_PIN = GPIO_NUM_39;           // Q of the 74HC74: 1 = overcurrent latched
static const gpio_num_t SCOPE_PIN = GPIO_NUM_16;           // free pin for an oscilloscope (25 is S88 PS here)
// Current measurement: 0.25 ohm, i.e. half of the 1150 mV/A measured on the
// prototype with 0.5 ohm (branch courant, 12 Nov 2026). To be calibrated on
// the board as on the prototype. Software threshold under the hardware one
// (4.1 A, LM393), and under the saturation of the measurement.
#define BOARD_CURRENT_MV_PER_A 575
#define BOARD_MAX_CURRENT_MA 3000

#else

#define BOARD_NAME "prototype"
static const gpio_num_t IN1_PIN = GPIO_NUM_27;
static const gpio_num_t IN2_PIN = GPIO_NUM_33;
static const gpio_num_t EN_PIN = GPIO_NUM_32;
static const gpio_num_t CURRENT_PIN = GPIO_NUM_36;
static const gpio_num_t FEEDBACK_PIN_BOARD = GPIO_NUM_26;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_4;
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_5;
static const gpio_num_t CUTOUT_PIN = GPIO_NUM_NC;          // no RailCom cutout
static const gpio_num_t TRK_RST_PIN = GPIO_NUM_NC;         // no hardware protection
static const gpio_num_t FAULT_PIN = GPIO_NUM_NC;
static const gpio_num_t SCOPE_PIN = GPIO_NUM_25;

#endif

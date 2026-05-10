#ifndef NANOBIT_ANALOG_CPP
#define NANOBIT_ANALOG_CPP

#include <Arduino.h>
#include "Nanobit_analog.h"

/*
 * Nanobit_analog.cpp — Single-word (32-bit) state to keep linker alignment quiet.
 *
 * State packing (little-endian friendly):
 *   bit [7:0]   : knob pin (uint8_t), default 2
 *   bit [31:8]  : lastValue (reserved, signed 24-bit if ever needed)
 */

// Default state: pin = 2 (0x02), lastValue = 0 → 0x00000200
// static uint32_t __gAnalog32 = 0x00000200u; // pin=2, last=0
static uint32_t __gAnalog32 __attribute__((aligned(8))) = ((uint32_t)(2u & 0xFFu)) | 0x00000000u;

static inline uint8_t __get_knob_pin()
{
  return (uint8_t)(__gAnalog32 & 0xFFu);
}
static inline void __set_knob_pin(uint8_t p)
{
  __gAnalog32 = (__gAnalog32 & 0xFFFFFF00u) | (uint32_t)p;
}
static inline int __get_lastValue()
{
  return (int)((int32_t)__gAnalog32) >> 8;
}
static inline void __set_lastValue(int v)
{
  __gAnalog32 = ((uint32_t)(v) << 8) | (uint32_t)(__gAnalog32 & 0xFFu);
  (void)__get_lastValue; // silence unused if not referenced
  (void)__set_lastValue;
}

/* -------------------------------------------------------
 * Optional lightweight ADC backend for nRF51 (register-level)
 * Enable with: #define NANOBIT_ANALOG_USE_LITE_ADC 1
 * Notes:
 *  - Uses NRF_ADC (10-bit). Minimal code size; no big ADC libraries.
 *  - Returns:
 *      0..1023 : valid conversion
 *      -1      : invalid/not mappable pin
 *      -2      : timeout waiting for END event
 * ------------------------------------------------------*/
#if NANOBIT_ANALOG_USE_LITE_ADC

// Nordic register/bitfield defs
#if defined(NRF51) || defined(NRF51_SERIES) || defined(NRF5) || defined(NRF51_S110)
#include <nrf.h>
#else
#warning "Lite ADC enabled on a non-nRF51 target; please ensure <nrf.h> and NRF_ADC exist."
#include <nrf.h> // try anyway; adjust for your core
#endif

// Arduino-nRF pin map symbol (provided by many nRF5 Arduino cores)
extern "C"
{
  extern const uint32_t g_ADigitalPinMap[]; // declared extern; defined by core
}

// Map nRF GPIO P0.xx to ADC AIN index (0..7) — common mapping on nRF51822
static inline int8_t nrfPinToAIN(uint32_t nrfpin)
{
  // On many nRF51 boards: AIN0..AIN7 ⇔ P0.01..P0.08 (check board variant if needed)
  if (nrfpin >= 1 && nrfpin <= 8)
    return (int8_t)(nrfpin - 1);
  return -1;
}

static inline void adcDisable_nRF51()
{
  NRF_ADC->TASKS_STOP = 1;
  NRF_ADC->ENABLE = (ADC_ENABLE_ENABLE_Disabled << ADC_ENABLE_ENABLE_Pos);
}

static int analogReadLiteAIN(uint8_t ain /*0..7*/)
{
  if (ain > 7)
    return -1;

  // Configure: 10-bit, 1/3 prescaling (~3.6V FS), VBG=1.2V reference
  uint32_t cfg = 0;
  cfg |= (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos);
  cfg |= (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos);
  cfg |= (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos);
  cfg |= ((ADC_CONFIG_PSEL_AnalogInput0 + ain) << ADC_CONFIG_PSEL_Pos);

  adcDisable_nRF51();
  NRF_ADC->CONFIG = cfg;
  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->ENABLE = (ADC_ENABLE_ENABLE_Enabled << ADC_ENABLE_ENABLE_Pos);

  NRF_ADC->TASKS_START = 1;

  // Busy-wait with a short timeout to avoid lockup in abnormal cases
  uint32_t t0 = micros();
  while (NRF_ADC->EVENTS_END == 0)
  {
    if ((micros() - t0) > 2000UL)
    { // >2ms → something is wrong
      adcDisable_nRF51();
      return -2;
    }
  }

  int result = (int)(NRF_ADC->RESULT & 0x3FF); // 10-bit
  adcDisable_nRF51();
  return result; // 0..1023
}

static int analogReadLitePin(uint8_t arduinoPin)
{
  // Convert Arduino pin → nRF GPIO number
  uint32_t nrfpin = g_ADigitalPinMap[arduinoPin];
  int8_t ain = nrfPinToAIN(nrfpin);
  if (ain < 0)
    return -1;
  return analogReadLiteAIN((uint8_t)ain);
}

#endif // NANOBIT_ANALOG_USE_LITE_ADC

/* -------------------------------------------------------
 * Public API
 * -----------------------------------------------------*/
int analog(uint8_t pinAN)
{
  int raw;
#if NANOBIT_ANALOG_USE_LITE_ADC
  raw = analogReadLitePin((uint8_t)pinAN);
#else
  raw = analogRead(pinAN);
#endif
  return raw;
}

int knob()
{
  int raw;
  raw =
#if NANOBIT_ANALOG_USE_LITE_ADC
      analogReadLitePin(__get_knob_pin());
#else
      analogRead(__get_knob_pin());
#endif

  // If lite-ADC reports an error (<0), treat as 0 for safety.
  if (raw < 0)
    return 0;

  // Clip low-end noise: anything 2 or below becomes 0.
  if (raw <= 2)
    return 0;

  // Scale linearly: (raw-2) * 1023 / 1021 → 0..1023
  long num = (long)(raw - 2) * 1023L;
  long denom = 1021L;
  int v = (int)(num / denom);

  // Final guardrails.
  if (v < 0)
    v = 0;
  if (v > 1023)
    v = 1023;
  return v;
}

int knob(int scale)
{
  long v = knob();
  // if (raw <= 0) return 0; // error passthrough

  // Scale to [0..scale], inclusive
  v = (v * (scale + 1L)) / 1023L;
  if (v > scale)
    v = scale;
  return (int)v;
}

int knob(int scaleCCW, int scaleCW)
{
  long v = knob();
  // if (raw <= 0) return 0; // error passthrough

  if (scaleCW >= scaleCCW)
  {
    long span = (long)((scaleCW + 1) - scaleCCW);
    if (span <= 0)
      return (int)scaleCCW;
    v = v / (1023L / span);
    v += scaleCCW;
    if (v > scaleCW)
      v = scaleCW;
  }
  else
  {
    // Reversed range
    long span = (long)((scaleCCW + 1) - scaleCW);
    if (span <= 0)
      return (int)scaleCW;
    v = 1023L - v;
    v = v / (1023L / span);
    v += scaleCW;
    if (v > scaleCCW)
      v = scaleCCW;
  }
  return (int)v;
}

void setKnobPin(uint8_t pin) { __set_knob_pin(pin); }
uint8_t getKnobPin() { return __get_knob_pin(); }

#endif // NANOBIT_ANALOG_CPP

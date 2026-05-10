#ifndef NANOBIT_ANALOG_H
#define NANOBIT_ANALOG_H

#include <Arduino.h>

/*
 * Nanobit_analog.h — Minimal-footprint analog helpers for micro:bit / nRF51 targets.
 *
 * NOTE ABOUT LINKAGE:
 * Some umbrella headers wrap their includes with `extern "C" { ... }`.
 * That breaks C++ overloading. To be resilient, we explicitly force C++ linkage
 * here so that knob() overloads remain valid even inside an outer `extern "C"`.
 */

#ifndef NANOBIT_ANALOG_USE_LITE_ADC
#define NANOBIT_ANALOG_USE_LITE_ADC 0
#endif

#ifdef __cplusplus
extern "C++" {
#endif

// Read raw analog value from an "analog pin" (0..1023 on 10-bit backends).
int analog(uint8_t pinAN);

// Read knob (uses internal knob pin; default = 2) mapped 2..1023 → 0..1023.
int knob();

// Read knob scaled to [0..scale] (inclusive).
int knob(int scale);

// Read knob scaled to [scaleCCW .. scaleCW] (inclusive), handles reversed ranges.
int knob(int scaleCCW, int scaleCW);

// Configure / query which pin is used for the knob helpers.
void    setKnobPin(uint8_t pin);
uint8_t getKnobPin(void);

#ifdef __cplusplus
} // extern "C++"
#endif

#endif // NANOBIT_ANALOG_H

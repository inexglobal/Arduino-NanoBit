#ifndef ANALOG_LITE_NRF51_H
#define ANALOG_LITE_NRF51_H

/*
 * AnalogLite_nRF51.h — Lightweight ADC backend for nRF51 (NRF_ADC, 10-bit)
 *
 * Purpose:
 *   - Keep code size small by avoiding large ADC modules from the Arduino core.
 *   - Provide pin→AIN mapping (commonly P0.01..P0.08 -> AIN0..AIN7 on nRF51822).
 *
 * Return values:
 *   - analogReadLiteAIN / analogReadLitePin:
 *       0..1023 : valid conversion (10-bit)
 *       -1      : invalid/not mappable pin
 *       -2      : timeout waiting for END event
 *
 * Usage:
 *   - Define NANOBIT_ANALOG_USE_LITE_ADC = 1 to use these functions in your wrapper.
 *   - Include this header from your Nanobit_analog.cpp (recommended) or any TU.
 *   - All functions are static inline to avoid ODR/multiple-definition issues.
 */

#if NANOBIT_ANALOG_USE_LITE_ADC

  // Nordic register headers (nRF51)
  #if defined(NRF51) || defined(NRF51_SERIES) || defined(NRF5) || defined(NRF51_S110)
    #include <nrf.h>
  #else
    // If you are not on nRF51 but force-enable lite ADC, we try to include <nrf.h> anyway.
    // Adjust as needed for your core.
    #include <nrf.h>
    #warning "AnalogLite_nRF51.h included on a non-nRF51 target. Ensure NRF_ADC exists."
  #endif

  // Provided by many Arduino nRF5 cores
  extern "C" {
    extern const uint32_t g_ADigitalPinMap[];
  }

  // Map nRF P0.xx -> AIN index (0..7). Common mapping: P0.01..P0.08 -> AIN0..AIN7
  static inline int8_t nrfPinToAIN(uint32_t nrfpin)
  {
    if (nrfpin >= 1 && nrfpin <= 8) return (int8_t)(nrfpin - 1);
    return -1;
  }

  static inline void adcDisable_nRF51(void)
  {
    NRF_ADC->TASKS_STOP = 1;
    NRF_ADC->ENABLE     = (ADC_ENABLE_ENABLE_Disabled << ADC_ENABLE_ENABLE_Pos);
  }

  static inline int analogReadLiteAIN(uint8_t ain /*0..7*/)
  {
    if (ain > 7) return -1;

    // Configure: 10-bit, 1/3 prescaling (~3.6V FS), VBG=1.2V reference
    uint32_t cfg = 0;
    cfg |= (ADC_CONFIG_RES_10bit                            << ADC_CONFIG_RES_Pos);
    cfg |= (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos);
    cfg |= (ADC_CONFIG_REFSEL_VBG                           << ADC_CONFIG_REFSEL_Pos);
    cfg |= ((ADC_CONFIG_PSEL_AnalogInput0 + ain)            << ADC_CONFIG_PSEL_Pos);

    adcDisable_nRF51();
    NRF_ADC->CONFIG     = cfg;
    NRF_ADC->EVENTS_END = 0;
    NRF_ADC->ENABLE     = (ADC_ENABLE_ENABLE_Enabled << ADC_ENABLE_ENABLE_Pos);

    NRF_ADC->TASKS_START = 1;

    // Wait with a short timeout to avoid lockup
    uint32_t t0 = micros();
    while (NRF_ADC->EVENTS_END == 0) {
      if ((micros() - t0) > 2000UL) { // >2ms indicates a fault
        adcDisable_nRF51();
        return -2;
      }
    }

    int result = (int)(NRF_ADC->RESULT & 0x3FF); // 10-bit
    adcDisable_nRF51();
    return result; // 0..1023
  }

  static inline int analogReadLitePin(uint8_t arduinoPin)
  {
    uint32_t nrfpin = g_ADigitalPinMap[arduinoPin];
    int8_t ain = nrfPinToAIN(nrfpin);
    if (ain < 0) return -1;
    return analogReadLiteAIN((uint8_t)ain);
  }

#else // NANOBIT_ANALOG_USE_LITE_ADC == 0

  // Stub out symbols if lite ADC is disabled to prevent accidental usage
  static inline int8_t nrfPinToAIN(uint32_t)                 { return -1; }
  static inline void   adcDisable_nRF51(void)                {}
  static inline int    analogReadLiteAIN(uint8_t)            { return -1; }
  static inline int    analogReadLitePin(uint8_t)            { return -1; }

#endif // NANOBIT_ANALOG_USE_LITE_ADC

#endif // ANALOG_LITE_NRF51_H

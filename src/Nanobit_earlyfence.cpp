// Nanobit_earlyfence.cpp
#include <nrf.h>

// extern "C" volatile int NB_EARLY_PROBE = 0;
// Alt hook: user can provide a strong symbol in sketch to override this.
extern "C" bool nanobit_allow_earlyfence(void) __attribute__((weak));
extern "C" bool nanobit_allow_earlyfence(void) { return true; }

// If either macro disables it, or the hook says false => skip.
#if !defined(NANOBIT_DISABLE_EARLYFENCE)

static void nanobit_early_servo_fence(void) __attribute__((constructor(200)));
static void nanobit_early_servo_fence(void) {
//   NB_EARLY_PROBE = 123;        // set probe
  if (!nanobit_allow_earlyfence()) return;

//   NB_EARLY_PROBE = 456;        // set probe
  __disable_irq();
  NRF_TIMER2->TASKS_STOP  = 1;
  NRF_TIMER2->TASKS_CLEAR = 1;
  NRF_TIMER2->EVENTS_COMPARE[0] = 0;
  NVIC_DisableIRQ(TIMER2_IRQn);
  NVIC_ClearPendingIRQ(TIMER2_IRQn);
  __enable_irq();
  

//   NB_EARLY_PROBE = 789;        // set probe
}

#endif // !NANOBIT_DISABLE_EARLYFENCE

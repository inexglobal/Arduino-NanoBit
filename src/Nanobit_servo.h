#ifndef NANOBIT_SERVO_H
#define NANOBIT_SERVO_H
// ===== 4-DOF Servo (Event-Driven) on nRF5 TIMER2 — micro:bit V1 (nRF51822) =====
// Core: sandeepmistry/arduino-nRF5
// API: servoAttach(pin[,min,max]), servoCalibrate(pin,min,max),
//      servoWriteDeg(pin,deg), servoWriteUS(pin,us), servoDetach(pin)
// Special: SV_OFF => release/freewheel (no pulses). Next valid cmd auto-resumes.
// Default pins (micro:bit V1): 8, 12, 13, 14

#include <Arduino.h>
#include <nrf.h>

#ifdef __cplusplus
extern "C"
{
#endif
  // Provided by arduino-nRF5 core: Arduino digital pin -> NRF GPIO index
  extern const uint32_t g_ADigitalPinMap[];
#ifdef __cplusplus
}
#endif

// -----------------------------------------------------------------------------
// Config
// -----------------------------------------------------------------------------
#define SERVO_COUNT 4
static const uint32_t SERVO_FRAME_US = 20000; // 20 ms
static const uint16_t GAP_US = 300;           // gap between channels
#define SV_OFF -9999

// Release behavior when turning a channel off (after its falling edge):
enum ReleaseMode : uint8_t
{
  RELEASE_HIZ = 0,
  RELEASE_PULLDOWN = 1,
  RELEASE_FORCE_LOW = 2
};
// Recommended: PULLDOWN to avoid floating input
static const ReleaseMode SERVO_RELEASE_MODE = RELEASE_PULLDOWN;

// -----------------------------------------------------------------------------
// Small utils
// -----------------------------------------------------------------------------
static inline void irq_lock() { __disable_irq(); }
static inline void irq_unlock() { __enable_irq(); }

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
  return (v < lo) ? lo : (v > hi ? hi : v);
}
static inline uint32_t pin2nrf(uint8_t arduinoPin)
{
  return g_ADigitalPinMap[arduinoPin];
}

// Fast GPIO in ISR
static inline void fast_set(uint32_t nrfpin) { NRF_GPIO->OUTSET = (1UL << nrfpin); }
static inline void fast_clear(uint32_t nrfpin) { NRF_GPIO->OUTCLR = (1UL << nrfpin); }

// Configure pin as input with pulldown (direct registers)
static inline void set_input_pulldown(uint32_t nrfpin)
{
  NRF_GPIO->PIN_CNF[nrfpin] =
      (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

// -----------------------------------------------------------------------------
// Front buffer (main thread writes)
// -----------------------------------------------------------------------------
struct __attribute__((aligned(8))) ServoInfo
{
  uint8_t edge; // Arduino pin
  uint8_t _pad1;
  uint16_t min_us;
  uint16_t max_us;
  uint16_t pulse_us;
  bool attached;
  bool muted; // true => released (no pulses)
  uint8_t _pad2;
};
alignas(8) static ServoInfo servos[SERVO_COUNT];

// -----------------------------------------------------------------------------
// Shadow snapshot (ISR reads only within a frame)
// -----------------------------------------------------------------------------
struct ServoShadow
{
  uint8_t edge;
  uint8_t _pad1;
  uint16_t pulse_us;
  uint32_t nrfpin;
  bool on; // attached && !muted
  uint8_t _pad2[3];
};
alignas(8) static ServoShadow sh[SERVO_COUNT];
alignas(8) static uint8_t act_idx[SERVO_COUNT]; // indices of active channels
alignas(8) static volatile uint8_t s_active_n = 0;
alignas(8) static volatile bool s_dirty = false; // main updated front buffer

// pending release flags: bit i = release servo i after its falling edge
alignas(8) static volatile uint8_t s_pending_release = 0;

// -----------------------------------------------------------------------------
// Timer/ISR state & stats
// -----------------------------------------------------------------------------
alignas(8) static volatile bool s_running = false;
alignas(8) static volatile uint32_t s_used_us = 0;
alignas(8) static volatile uint16_t s_stage = 0; // 0..(active_n*2-1)
alignas(8) static volatile uint32_t s_overruns = 0;
alignas(8) static volatile uint32_t s_max_used = 0;

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------
static void gpio_cfg_output_if_needed(uint8_t arduinoPin)
{
  // Not time-critical: safe to use Arduino API outside ISR
  pinMode(arduinoPin, OUTPUT);
  digitalWrite(arduinoPin, LOW); // idle LOW between pulses
}
static void gpio_cfg_hiz(uint8_t arduinoPin)
{
  pinMode(arduinoPin, INPUT); // Hi-Z (no pull)
}

static void rebuild_shadow_unlocked()
{
  // Called with IRQs disabled (from ISR start-of-frame) or in a safe section
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < SERVO_COUNT; i++)
  {
    sh[i].edge = servos[i].edge;
    sh[i].pulse_us = servos[i].pulse_us;
    sh[i].nrfpin = pin2nrf(servos[i].edge);
    sh[i].on = (servos[i].attached && !servos[i].muted);
    if (sh[i].on)
      act_idx[cnt++] = i;
  }
  s_active_n = cnt;
}

static inline void timer2_program_cc0(uint16_t us) { NRF_TIMER2->CC[0] = us; }

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool servoAttach(uint8_t edge_pin, uint16_t min_us = 500, uint16_t max_us = 2500)
{
  if (min_us > max_us)
  {
    uint16_t t = min_us;
    min_us = max_us;
    max_us = t;
  }

  irq_lock();
  for (int i = 0; i < SERVO_COUNT; i++)
  {
    if (!servos[i].attached)
    {
      servos[i].edge = edge_pin;
      servos[i].min_us = min_us;
      servos[i].max_us = max_us;
      servos[i].pulse_us = (uint16_t)((min_us + max_us) / 2);
      servos[i].muted = true; // start released
      servos[i].attached = true;
      s_dirty = true;
      irq_unlock();

      gpio_cfg_hiz(edge_pin);
      return true;
    }
  }
  irq_unlock();
  return false;
}

bool servoDetach(uint8_t edge_pin)
{
  bool ok = false;
  irq_lock();
  for (int i = 0; i < SERVO_COUNT; i++)
  {
    if (servos[i].attached && servos[i].edge == edge_pin)
    {
      servos[i].attached = false;
      servos[i].muted = true;
      s_pending_release &= ~(1u << i); // clear pending
      s_dirty = true;
      ok = true;
      break;
    }
  }
  irq_unlock();
  if (ok)
    gpio_cfg_hiz(edge_pin);
  return ok;
}
bool servoStop(uint8_t edge_pin){servoDetach(edge_pin);}
void servoCalibrate(uint8_t edge_pin, uint16_t min_us, uint16_t max_us)
{
  if (min_us > max_us)
  {
    uint16_t t = min_us;
    min_us = max_us;
    max_us = t;
  }
  irq_lock();
  for (int i = 0; i < SERVO_COUNT; i++)
  {
    if (servos[i].attached && servos[i].edge == edge_pin)
    {
      servos[i].min_us = min_us;
      servos[i].max_us = max_us;
      servos[i].pulse_us = clamp_u16(servos[i].pulse_us, min_us, max_us);
      s_dirty = true;
      break;
    }
  }
  irq_unlock();
}

// Write µs or SV_OFF
void servoMicroseconds(uint8_t edge_pin, int us)
{
  if (us == SV_OFF)
  {
    irq_lock();
    for (int i = 0; i < SERVO_COUNT; i++)
    {
      if (servos[i].attached && servos[i].edge == edge_pin)
      {
        servos[i].muted = true; // prevent future pulses
        s_dirty = true;
        s_pending_release |= (1u << i); // release after falling edge (in ISR)
        irq_unlock();
        return; // DO NOT change pinMode here (avoid mid-pulse cut)
      }
    }
    irq_unlock();
    return;
  }

  irq_lock();
  for (int i = 0; i < SERVO_COUNT; i++)
  {
    if (servos[i].attached && servos[i].edge == edge_pin)
    {
      if (servos[i].muted)
      {
        servos[i].muted = false;
        irq_unlock();
        gpio_cfg_output_if_needed(edge_pin); // safe outside lock
        irq_lock();
      }
      servos[i].pulse_us = clamp_u16((uint16_t)us, servos[i].min_us, servos[i].max_us);
      s_dirty = true;
      break;
    }
  }
  irq_unlock();
}

void servo(uint8_t edge_pin, float deg)
{
  if (deg == SV_OFF)
  {
    servoMicroseconds(edge_pin, SV_OFF);
    return;
  }
  if (deg < 0)
    deg = 0;
  if (deg > 180)
    deg = 180;

  irq_lock();
  for (int i = 0; i < SERVO_COUNT; i++)
  {
    if (servos[i].attached && servos[i].edge == edge_pin)
    {
      if (servos[i].muted)
      {
        servos[i].muted = false;
        irq_unlock();
        gpio_cfg_output_if_needed(edge_pin);
        irq_lock();
      }
      float span = (float)(servos[i].max_us - servos[i].min_us);
      uint16_t us = (uint16_t)(servos[i].min_us + (deg / 180.0f) * span + 0.5f);
      servos[i].pulse_us = clamp_u16(us, servos[i].min_us, servos[i].max_us);
      s_dirty = true;
      break;
    }
  }
  irq_unlock();
}

// -----------------------------------------------------------------------------
// TIMER2 ISR
// -----------------------------------------------------------------------------
extern "C" void TIMER2_IRQHandler(void)
{
  if (!NRF_TIMER2->EVENTS_COMPARE[0])
    return;
  NRF_TIMER2->EVENTS_COMPARE[0] = 0;

  // Start-of-frame: rebuild snapshot if dirty
  if (s_stage == 0)
  {
    if (s_dirty)
    {
      rebuild_shadow_unlocked();
      s_dirty = false;
    }
    s_used_us = 0;
  }

  const uint16_t stages_total = (uint16_t)(s_active_n << 1); // rise+fall per active
  if (s_stage < stages_total)
  {
    const uint8_t idx = (uint8_t)(s_stage >> 1);
    const bool rise = ((s_stage & 1) == 0);
    const uint8_t i = act_idx[idx];
    const uint32_t pin = sh[i].nrfpin;

    if (rise)
    {
      fast_set(pin);
      const uint16_t on = sh[i].pulse_us;
      timer2_program_cc0(on);
      s_used_us += on;
    }
    else
    {
      fast_clear(pin); // falling edge (safe point)
      timer2_program_cc0(GAP_US);
      s_used_us += GAP_US;

      // Handle deferred release AFTER falling edge
      if (s_pending_release & (1u << i))
      {
        s_pending_release &= ~(1u << i);
        switch (SERVO_RELEASE_MODE)
        {
        case RELEASE_FORCE_LOW:
          // Keep as OUTPUT LOW: nothing to change (already LOW)
          break;
        case RELEASE_PULLDOWN:
          set_input_pulldown(pin);
          break;
        case RELEASE_HIZ:
        default:
          // Plain Hi-Z (may float slightly on long wires)
          NRF_GPIO->PIN_CNF[pin] =
              (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
              (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
              (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
              (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
              (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
          break;
        }
      }
    }
    s_stage++;
  }
  else
  {
    // Frame tail
    uint32_t rest = (SERVO_FRAME_US > s_used_us) ? (SERVO_FRAME_US - s_used_us) : 5; // tiny rest on overrun
    if (s_used_us > s_max_used)
      s_max_used = s_used_us;
    if (rest == 5)
      s_overruns++;

    timer2_program_cc0((rest <= 0xFFFFu) ? (uint16_t)rest : 0xFFFFu);
    s_stage = 0;
  }

  NRF_TIMER2->TASKS_CLEAR = 1;
}

// -----------------------------------------------------------------------------
// TIMER2 control
// -----------------------------------------------------------------------------
static void timer2_start()
{
  if (s_running)
    return;
  s_running = true;
  s_stage = 0;
  s_used_us = 0;
  s_overruns = 0;
  s_max_used = 0;

  NRF_TIMER2->TASKS_STOP = 1;
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
  NRF_TIMER2->PRESCALER = 4; // 16 MHz / 2^4 = 1 MHz (1us/tick)

  NRF_TIMER2->EVENTS_COMPARE[0] = 0;
  NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos);
  NRF_TIMER2->SHORTS = 0;

  NVIC_SetPriority(TIMER2_IRQn, 3);
  NVIC_ClearPendingIRQ(TIMER2_IRQn);
  NVIC_EnableIRQ(TIMER2_IRQn);

  NRF_TIMER2->CC[0] = 1000; // first tick after 1 ms
  NRF_TIMER2->TASKS_CLEAR = 1;
  NRF_TIMER2->TASKS_START = 1;

  // Build initial snapshot for first frame
  irq_lock();
  rebuild_shadow_unlocked();
  s_dirty = false;
  irq_unlock();
}

static void timer2_stop()
{
  if (!s_running)
    return;
  NVIC_DisableIRQ(TIMER2_IRQn);
  NRF_TIMER2->TASKS_STOP = 1;
  s_running = false;

  // Put all attached pins to safe state
  for (int i = 0; i < SERVO_COUNT; i++)
  {
    if (servos[i].attached)
    {
      const uint32_t pin = pin2nrf(servos[i].edge);
      fast_clear(pin);
      // follow release mode for stop as well
      if (SERVO_RELEASE_MODE == RELEASE_FORCE_LOW)
      {
        // keep low as output (already low)
        pinMode(servos[i].edge, OUTPUT);
        digitalWrite(servos[i].edge, LOW);
      }
      else if (SERVO_RELEASE_MODE == RELEASE_PULLDOWN)
      {
        set_input_pulldown(pin);
      }
      else
      {
        gpio_cfg_hiz(servos[i].edge);
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Convenience
// -----------------------------------------------------------------------------
void initServo()
{
  // Default: attach common micro:bit V1 servo pins
  servoAttach(8);
  servoAttach(12);
  servoAttach(13);
  servoAttach(14);
  timer2_start();
}

void deinitServo()
{
  // Default: Detach common micro:bit V1 servo pins
  servoDetach(8);
  servoDetach(12);
  servoDetach(13);
  servoDetach(14);
  timer2_stop();
}

// Optional debug
uint32_t servoFrameUsedUs() { return s_used_us; }
uint32_t servoOverruns() { return s_overruns; }
uint32_t servoMaxUsedUs() { return s_max_used; }

#endif // NANOBIT_SERVO_H

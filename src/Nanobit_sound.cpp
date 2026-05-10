#ifndef NANOBIT_SOUND_CPP
#define NANOBIT_SOUND_CPP

#include <Arduino.h>
#include "Nanobit_sound.h"

#define PIEZO_PIN 0

void sound(uint16_t freqHz, uint32_t durMs)
{
  pinMode(PIEZO_PIN, OUTPUT);
  if (freqHz == 0 || durMs == 0)
  {
    noSound();
    return;
  }

  uint32_t half_us = 1000000UL / (freqHz * 2UL);
  uint32_t cycles = (uint32_t)freqHz * durMs / 1000UL;

  for (uint32_t i = 0; i < cycles; i++)
  {
    digitalWrite(PIEZO_PIN, HIGH);
    delayMicroseconds(half_us);
    digitalWrite(PIEZO_PIN, LOW);
    delayMicroseconds(half_us);
  }
}

void noSound()
{
  pinMode(PIEZO_PIN, OUTPUT);
  digitalWrite(PIEZO_PIN, LOW);
}

void beep(){
  sound(500,100);
}
#endif
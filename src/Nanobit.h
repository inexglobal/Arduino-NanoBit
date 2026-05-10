#ifndef NANOBIT_H
#define NANOBIT_H

#define LED_BLTIN_PIN 15

#define NANOBIT_DISABLE_EARLYFENCE

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include "Nanobit_sound.h"
#include "Nanobit_analog.h"
#include "Nanobit_in_out.h"
#include "Nanobit_OLED_I2C_SSD1306.h"
#include "Nanobit_servo.h"
#include "Nanobit_Trajectory.h"

alignas(8) SSD1306_EZ oled;

#endif
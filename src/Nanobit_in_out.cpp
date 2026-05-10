#ifndef NANOBIT_IN_OUT_CPP
#define NANOBIT_IN_OUT_CPP

#include <Arduino.h>
#include <Nanobit_in_out.h>

static uint8_t _SWApin = 5;
static uint8_t _SWBpin = 11;
//-------------------------------------------------------------
// Digital in,out
//-------------------------------------------------------------
int in(int p)
{
    pinMode(p, INPUT_PULLUP);
    return digitalRead(p);
}
void out(int p, int dat)
{
    pinMode(p, OUTPUT);
    digitalWrite(p, dat);
}
int SW_A()
{
    pinMode(_SWApin, INPUT_PULLUP);
    return (!digitalRead(_SWApin));
}
int SW_B()
{
    pinMode(_SWBpin, INPUT_PULLUP);
    return (!digitalRead(_SWBpin));
}
void SW_A_press()
{
    while (in(SW_A()) == 0)
        ;
    while (in(SW_A()))
        ;
}
void SW_B_press()
{
    while (SW_B() == 0)
        ;
    while (SW_B())
        ;
}

#endif
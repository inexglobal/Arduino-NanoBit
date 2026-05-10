#ifndef NANOBIT_IN_OUT_H
#define NANOBIT_IN_OUT_H


#include <Arduino.h>

int in(int p);
void out(int p,int dat);

int SW_A();
int SW_B();
void SW_A_press();
void SW_B_press();

#define sw_a SW_A
#define sw_A SW_A
#define sw_a_press	SW_A_press
#define sw_A_press	SW_A_press

#define sw_b sw_B
#define sw_B SW_B
#define sw_b_press	SW_B_press
#define sw_B_press	SW_B_press

#endif
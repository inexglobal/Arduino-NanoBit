#ifndef NANOBIT_SOUND_H
#define NANOBIT_SOUND_H

/*************************************************
 * Public Constants
 *************************************************/
#define B_0 31
#define C_1 33
#define CS_1 35
#define D_1 37
#define DS_1 39
#define E_1 41
#define F_1 44
#define FS_1 46
#define G_1 49
#define GS_1 52
#define A_1 55
#define AS_1 58
#define B_1 62
#define C_2 65
#define CS_2 69
#define D_2 73
#define DS_2 78
#define E_2 82
#define F_2 87
#define FS_2 93
#define G_2 98
#define GS_2 104
#define A_2 110
#define AS_2 117
#define B_2 123
#define C_3 131
#define CS_3 139
#define D_3 147
#define DS_3 156
#define E_3 165
#define F_3 175
#define FS_3 185
#define G_3 196
#define GS_3 208
#define A_3 220
#define AS_3 233
#define B_3 247
#define C_4 262
#define CS_4 277
#define D_4 294
#define DS_4 311
#define E_4 330
#define F_4 349
#define FS_4 370
#define G_4 392
#define GS_4 415
#define A_4 440
#define AS_4 466
#define B_4 494
#define C_5 523
#define CS_5 554
#define D_5 587
#define DS_5 622
#define E_5 659
#define F_5 698
#define FS_5 740
#define G_5 784
#define GS_5 831
#define A_5 880
#define AS_5 932
#define B_5 988
#define C_6 1047
#define CS_6 1109
#define D_6 1175
#define DS_6 1245
#define E_6 1319
#define F_6 1397
#define FS_6 1480
#define G_6 1568
#define GS_6 1661
#define A_6 1760
#define AS_6 1865
#define B_6 1976
#define C_7 2093
#define CS_7 2217
#define D_7 2349
#define DS_7 2489
#define E_7 2637
#define F_7 2794
#define FS_7 2960
#define G_7 3136
#define GS_7 3322
#define A_7 3520
#define AS_7 3729
#define B_7 3951
#define C_8 4186
#define CS_8 4435
#define D_8 4699
#define DS_8 4978

#include <Arduino.h>

void sound(uint16_t freqHz, uint32_t durMs);
void noSound();
void beep();

#endif
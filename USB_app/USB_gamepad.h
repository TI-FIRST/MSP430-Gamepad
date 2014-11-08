/*
 * gamepad.h
 *
 *  Created on: Sep 29, 2014
 *      Author: a0868333
 */

#ifndef GAMEPAD_H_
#define GAMEPAD_H_

#include <stdint.h>

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t ar;
    int16_t bx;
    int16_t by;
    int16_t bz;
    int16_t br;
    uint32_t buttons;
} USB_GamepadReportTx;

typedef struct
{
    uint16_t indicators;
}USB_GamepadReportRx;

#endif /* GAMEPAD_H_ */

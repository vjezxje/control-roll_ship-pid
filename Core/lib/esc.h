/*
 * esc.h
 *
 *  Created on: Jun 20, 2024
 *      Author: Hp
 */

#ifndef LIB_ESC_H_
#define LIB_ESC_H_
#include <stdint.h>
#include <stdio.h>
#include "math.h"


void esc_init();
void calib_esc();
void MotorDriver (float duty);
#endif /* LIB_ESC_H_ */

/*
 * design by Jacno
 *
 */

#include "stdint.h"
#include "string.h"
#include "stdio.h"

typedef struct _data_t{

			uint8_t mode;
			uint8_t clock;
			float eul_roll_deg;
			float eul_pitch_deg;
			float acc_x_filter ;
			float omega_filter;
			float angle;
			float angle_pendulum;
			float SP_angle;
			float gyr_y_deg;
		} data_t;
void _pack_data(uint8_t *data_array_out, data_t data);
void _unpack_data(const uint8_t *data_array_in, data_t* data_in);


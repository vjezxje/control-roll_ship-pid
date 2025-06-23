/*
 * design by Jacno
 *
 */

#include "config_data.h"

void _pack_data(uint8_t *data_array_out, data_t data)
{
	// Gán giá trị float vào mảng
	float  _eul_roll_deg, _eul_pitch_deg, _acc_x_filter, _omega_filter, _angle, _angle_pendulum, _SP_angle, _gyr_y_deg;
	_eul_roll_deg 		= data.eul_roll_deg;
	_eul_pitch_deg 		= data.eul_pitch_deg;
	_acc_x_filter 		= data.acc_x_filter ;
	_omega_filter 		= data.omega_filter;
	_angle 				= data.angle;
	_angle_pendulum 	= data.angle_pendulum;
	_SP_angle			= data.SP_angle;
	_gyr_y_deg 			= data.gyr_y_deg;

	memcpy(&data_array_out[0],  &_eul_roll_deg , sizeof(float ));
	memcpy(&data_array_out[4],  &_eul_pitch_deg, sizeof(float));
	memcpy(&data_array_out[8],  &_acc_x_filter, sizeof(float));
	memcpy(&data_array_out[12], &_omega_filter, sizeof(float));
    memcpy(&data_array_out[16], &_angle , sizeof(float));
    memcpy(&data_array_out[20], &_angle_pendulum, sizeof(float));
	memcpy(&data_array_out[24], &_SP_angle, sizeof(float));
	memcpy(&data_array_out[28], &_gyr_y_deg , sizeof(float ));
}
void _unpack_data(const uint8_t *data_array_in, data_t* data_in)
{
	float  _eul_roll_deg, _eul_pitch_deg, _acc_x_filter, _omega_filter, _angle, _angle_pendulum, _SP_angle, _gyr_y_deg;
    memcpy(&_eul_roll_deg, 	&data_array_in[0], 	sizeof(float));
    memcpy(&_eul_pitch_deg, 	&data_array_in[4], 	sizeof(float));
    memcpy(&_acc_x_filter, 	&data_array_in[8], 	sizeof(float));
	memcpy(&_omega_filter, 			&data_array_in[12], sizeof(float));
    memcpy(&_angle, 			&data_array_in[16], sizeof(float));
	memcpy(&_angle_pendulum,  &data_array_in[20], sizeof(float));
	memcpy(&_SP_angle, 	&data_array_in[24], sizeof(float));
	memcpy(&_gyr_y_deg,  &data_array_in[28], sizeof(float));

	data_in->	mode 				= 	data_array_in[0];
	data_in->	eul_roll_deg 		= 	_eul_roll_deg;
	data_in->	eul_pitch_deg	    = 	_eul_pitch_deg;
	data_in->	acc_x_filter	    = 	_acc_x_filter;
	data_in->	omega_filter 		=	_omega_filter;
	data_in->	angle				=	_angle;
	data_in->	angle_pendulum 		=	_angle_pendulum;
	data_in->	SP_angle 			=	_SP_angle;
	data_in->	gyr_y_deg		=	_gyr_y_deg;
}

//eul_roll_deg;
//eul_pitch_deg;
//acc_x_filer ;
//omega_filter
//angle
//angle_pendulum
//omega_motor
//omega_G_Degrees


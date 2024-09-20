/*
 * Guidance.c
 *
 * Created: 2024-08-07 7:24:29 PM
 *  Author: mjk32
 */ 

#include <math.h>

float Theta1(float* q, float v[3])
{	
	
	float ww = q[0] * q[0];
	float xx = q[1] * q[1];
	float yy = q[2] * q[2];
	float zz = q[3] * q[3];
	float wx = q[0] * q[1];
	float wy = q[0] * q[2];
	float wz = q[0] * q[3];
	float xy = q[1] * q[2];
	float xz = q[1] * q[3];
	float yz = q[2] * q[3];
	
	return atan2((2*xy*v[0] + yy*v[1] + 2*yz*v[2] + 2*wz*v[0] - zz*v[1] + ww*v[1] - 2*wx*v[2] - xx*v[1])
	,(ww*v[0] + 2*wy*v[2] - 2*wz*v[1] + xx*v[0] + 2*xy*v[1] + 2*xz*v[2] - zz*v[0] - yy*v[0]));
}

float Theta2(float v[2], float w[2])
{
	return atan2((v[1]-w[1]), (v[0]-w[0]));
}
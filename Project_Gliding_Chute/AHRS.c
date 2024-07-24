/*
 * AHRS.c
 *
 * Created: 2024-07-24 오후 5:31:14
 *  Author: stu11
 */ 

#include <math.h>

#define beta 1.5
#define deltat 1/64

void Quaternion_rotate(float* q, float v[3], float output[3])
{
	float result[3];

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
	
	result[0] = ww*v[0] + 2*wy*v[2] - 2*wz*v[1] +
	xx*v[0] + 2*xy*v[1] + 2*xz*v[2] -
	zz*v[0] - yy*v[0];
	result[1] = 2*xy*v[0] + yy*v[1] + 2*yz*v[2] +
	2*wz*v[0] - zz*v[1] + ww*v[1] -
	2*wx*v[2] - xx*v[1];
	result[2] = 2*xz*v[0] + 2*yz*v[1] + zz*v[2] -
	2*wy*v[0] - yy*v[2] + 2*wx*v[1] -
	xx*v[2] + ww*v[2];

	// Copy result to output
	output[0] = result[0];
	output[1] = result[1];
	output[2] = result[2];
}

void Quaternion_multiply(float q1[3], float q2[3], float output[3])
{
    output[0] = q1[0]   *q2[0]    - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    output[1] = q1[1]*q2[0]    + q1[0]   *q2[1] + q1[2]*q2[3] - q1[3]*q2[2];
    output[2] = q1[0]   *q2[2] - q1[1]*q2[3] + q1[2]*q2[0]    + q1[3]*q2[1];
    output[3] = q1[0]   *q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]   ;
}



void AHRS_Init(float* q, float ax, float ay, float az, float mx, float my, float mz)
{
	float AccelNorm = sqrt(ax * ax + ay * ay + az * az);
	float MagNorm = sqrt(mx * mx + my * my + mz * mz);
	float NormalizedAccel[3], NormalizedMag[3], PredictedMagneticfield[3], q_delta[3];
	float Gamma;
	NormalizedAccel[0] = ax / AccelNorm;
	NormalizedAccel[1] = ay / AccelNorm;
	NormalizedAccel[2] = az / AccelNorm;
	NormalizedMag[0] = mx / MagNorm;
	NormalizedMag[1] = my / MagNorm;
	NormalizedMag[2] = mz / MagNorm;

	if (NormalizedAccel[2] > 0)
	{
		q[0] = sqrt((NormalizedAccel[2] + 1) / 2);
		q[1] = NormalizedAccel[1] / sqrt(2 * (NormalizedAccel[2] + 1));
		q[2] = -NormalizedAccel[0] / sqrt(2 * (NormalizedAccel[2] + 1));
		q[3] = 0;
	}
	else
	{
		q[0] = -NormalizedAccel[1] / sqrt(2 * (-NormalizedAccel[2] + 1));
		q[1] = -sqrt((-NormalizedAccel[2] + 1) / 2);
		q[2] = 0;
		q[3] = -NormalizedAccel[0] / sqrt(2 * (NormalizedAccel[2] + 1));
	}

	Quaternion_rotate(q, NormalizedMag, PredictedMagneticfield);
	Gamma = PredictedMagneticfield[0] * PredictedMagneticfield[0] + PredictedMagneticfield[1] * PredictedMagneticfield[1];
	
	
	if (NormalizedMag[0] > 0)
	{
		q_delta[0] = sqrt(Gamma + PredictedMagneticfield[0] * sqrt(Gamma)) / sqrt(2 * Gamma);
		q_delta[1] = 0;
		q_delta[2] = 0;
		q_delta[3] = PredictedMagneticfield[1] / sqrt(2 * (Gamma + PredictedMagneticfield[0] * sqrt(Gamma)));

	}
	else
	{
		q_delta[0] = PredictedMagneticfield[1] / sqrt(2 * (Gamma - PredictedMagneticfield[0] * sqrt(Gamma)));
		q_delta[1] = 0;
		q_delta[2] = 0;
		q_delta[3] = sqrt(Gamma - PredictedMagneticfield[0] * sqrt(Gamma)) / sqrt(2 * Gamma);
	}
	q[0] = -q[0];
	q[1] = -q[1];
	q[2] = -q[2];
	q[3] = -q[3];
	Quaternion_multiply(q, q_delta, q);
}

void MadgwickQuaternionUpdate(float* q, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{	
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}
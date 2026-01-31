/*
 * Calculations.c
 *
 *  Created on: Dec 15, 2025
 *      Author: matthias Ellinger
 */




#include "calculations.h"
#include <math.h>

#define Integration_cap 400
#define Looptime 0.002 //seconds between measurements
#define gyro_error 4
#define acc_error 2

void pid_equation(float Error, float Prev_Error, float Prev_Int, float P, float I, float D, float* PID_Output)
{
	float Pterm = Error * P;
	float Iterm = Prev_Int + (I *(Error + Prev_Error)*Looptime)/2;
	if (Iterm > Integration_cap)
	{
		Iterm = Integration_cap;
	}
	else if(Iterm <(-Integration_cap))
	{
		Iterm = Integration_cap;
	}
	float Dterm = D*(Error + Prev_Error)/Looptime;
	float PIDOut = Pterm+ Iterm + Dterm;
	if (PIDOut > Integration_cap)
		{
			PIDOut = Integration_cap;
		}
		else if(PIDOut <(-Integration_cap))
		{
			PIDOut = Integration_cap;
		}
	*PID_Output = PIDOut;
	*(PID_Output+1) = Iterm;

}

int pwm_cap(int pwm)
{
	if(pwm >= 300)
	{
		pwm = 299;
	}
	if(pwm < 0)
		{
			pwm = 0;
		}
	return pwm;
}

float acc_roll(float acc_x,float acc_y, float acc_z)
{
	float acc_roll = 180*atan((double)(acc_y/sqrtf((acc_x*acc_x)+(acc_z*acc_z))))/M_PI;
	return acc_roll;
}

float acc_pitch(float acc_x,float acc_y, float acc_z)
{
	float acc_pitch = 180*atan((double)(-acc_x/sqrtf((acc_y*acc_y)+(acc_z*acc_z))))/M_PI;
	return acc_pitch;
}


void KalmanCalculation(float State, float Uncertainty, float Input, float Measurement, float* KalmanOutput)
{
	State = State + Looptime*Input;
	Uncertainty = Uncertainty + Looptime*Looptime*gyro_error*gyro_error;
	float Gain = Uncertainty/(Uncertainty+acc_error*acc_error);
	State= State + Gain*(Measurement-State);
	Uncertainty = (1-Gain)*Uncertainty;
	KalmanOutput[0] = State;
	KalmanOutput[1] = Uncertainty;
}



void motor_inputs(float InputRoll, float InputPitch, float InputYaw, int* MotorInput)
{
	float pwm_scale = 1;
	*(MotorInput+0) = (int)(pwm_scale*((+1)*InputPitch+(-1)*InputRoll+(+1)*InputYaw));
	*(MotorInput+1) = (int)(pwm_scale*((+1)*InputPitch+(+1)*InputRoll+(-1)*InputYaw));
	*(MotorInput+2) = (int)(pwm_scale*((-1)*InputPitch+(+1)*InputRoll+(+1)*InputYaw));
	*(MotorInput+3) = (int)(pwm_scale*((-1)*InputPitch+(-1)*InputRoll+(-1)*InputYaw));
}

float yaw_cap(float yaw)
{
	  if(yaw > 180)
	  {
		  yaw -= 360;
	  }
	  if(yaw <= -180)
	  {
		  yaw += 360;
	  }
	  return yaw;
}

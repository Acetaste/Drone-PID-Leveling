/*
 * Calculations.c
 *
 *  Created on: Dec 15, 2025
 *      Author: matthias Ellinger
 */




#include "calculations.h"
#include <math.h>

#define Cap 120
#define Loop_Time 2.5/1000 //seconds between measurements
#define gyro_error 4
#define acc_error 2
#define rad_to_degree 57.29578f
#define degree_to_rad 0.017453f

void pid_equation(float Error, float Prev_Error, float Prev_Int, float P, float I, float D, float* PID_Output)
{
	float Pterm = Error * P;
	float Iterm = Prev_Int + (I *(Error + Prev_Error)*Loop_Time)/2;
	if (Iterm > Cap)
	{
		Iterm = Cap;
	}
	else if(Iterm <(-Cap))
	{
		Iterm = -Cap;
	}
	float Dterm = D*(Error - Prev_Error)/Loop_Time;
	float PIDOut = Pterm+ Iterm + Dterm;

	*PID_Output = PIDOut;
	*(PID_Output+1) = Iterm;

}

int pwm_cap(int pwm)
{
	if(pwm > Cap)
	{
		pwm = Cap;
	}
	if(pwm < 0)
	{
		pwm = 0;
	}
	return pwm;
}

float acc_roll(float acc_x,float acc_y, float acc_z)
{
	float acc_roll = rad_to_degree*(atanf(acc_y/sqrtf((acc_x*acc_x)+(acc_z*acc_z))));

	if(acc_z >=0)
	{
		return acc_roll;
	}
	else
	{
		if(acc_y >= 0)
		{
			return 180 - acc_roll;
		}
		else
		{
			return (-180) - acc_roll;
		}
	}

}

float acc_pitch(float acc_x,float acc_y, float acc_z)
{
	float acc_pitch = rad_to_degree*(atanf(-acc_x/sqrtf((acc_y*acc_y)+(acc_z*acc_z))));
	if(acc_z >=0)
	{
		return acc_pitch;
	}
	else
	{
		if((-1*acc_x) >= 0)
		{
			return 180 - acc_pitch;
		}
		else
		{
			return (-180) - acc_pitch;
		}
	}
}


void KalmanCalculation(float State, float Uncertainty, float Input, float Measurement, float* KalmanOutput)
{
	State = State + Loop_Time*Input;
	Uncertainty = Uncertainty + Loop_Time*Loop_Time*gyro_error*gyro_error;
	float Gain = Uncertainty/(Uncertainty+acc_error*acc_error);
	State= State + Gain*(Measurement-State);
	Uncertainty = (1-Gain)*Uncertainty;
	KalmanOutput[0] = State;
	KalmanOutput[1] = Uncertainty;
}

void body_rate_to_fixed_rate(float* body_rate,float degree_phi, float degree_theta, float * degree_output)
{
	float rad_theta 	= 	degree_theta		* degree_to_rad;
	float rad_phi 		= 	degree_phi			* degree_to_rad;
	float rad_p			=	(*(body_rate+0))	* degree_to_rad;
	float rad_q			=	(*(body_rate+1))	* degree_to_rad;
	float rad_r			=	(*(body_rate+2))	* degree_to_rad;
	//roll
	*(degree_output+0) = rad_to_degree*(	rad_p 	+ 	rad_q * (sinf(rad_phi) * tanf(rad_theta)) 	+	rad_r * cosf(rad_phi)*tanf(rad_theta));
	//pitch
	*(degree_output+1) = rad_to_degree*(				rad_q * cosf(rad_phi) 						-	rad_r * sinf(rad_phi));
	//yaw
	*(degree_output+2) = rad_to_degree*(				rad_q * sinf(rad_phi) / cosf(rad_theta) 	+ 	rad_r * cosf(rad_phi)/cosf(rad_theta));
}


void motor_inputs(float InputPitch, float InputRoll, float InputYaw, int* MotorInput)
{
	float pwm_scale = 1;
	*(MotorInput+0) = (int)(pwm_scale*	((-1) * InputRoll		+ (+1) * InputPitch		+ (-1) * InputYaw));
	*(MotorInput+1) = (int)(pwm_scale*	((+1) * InputRoll		+ (+1) * InputPitch		+ (+1) * InputYaw));
	*(MotorInput+2) = (int)(pwm_scale*	((+1) * InputRoll		+ (-1) * InputPitch		+ (-1) * InputYaw));
	*(MotorInput+3) = (int)(pwm_scale*	((-1) * InputRoll		+ (-1) * InputPitch		+ (+1) * InputYaw));
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

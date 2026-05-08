/*
 * Calculations.c
 *
 *  Created on: Dec 15, 2025
 *      Author: matthias Ellinger
 */




#include "calculations.h"
#include "user_constants.h"
#include <math.h>


void pid_equation(float Error, float Prev_Error, float Prev_Int, float P, float I, float D, float* PID_Output)
{
	float Pterm = Error * P;
	float Iterm = Prev_Int + (I *(Error + Prev_Error)*LOOP_TIME_S)/2;
	if (Iterm > I_CAP)
	{
		Iterm = I_CAP;
	}
	else if(Iterm <(-I_CAP))
	{
		Iterm = -I_CAP;
	}


	float Dterm = D*(Error - Prev_Error)/LOOP_TIME_S;
	float PIDOut = Pterm+ Iterm + Dterm;

	if (PIDOut > PID_CAP)
	{
		PIDOut = PID_CAP;
	}
	else if(PIDOut <(-PID_CAP))
	{
		PIDOut = -PID_CAP;
	}
	*PID_Output = PIDOut;
	*(PID_Output+1) = Iterm;

}

int pwm_cap(int pwm)
{
	if(pwm > PWM_CAP)
	{
		pwm = PWM_CAP;
	}
	if(pwm < 0)
	{
		pwm = 0;
	}
	return pwm;
}

float acc_roll(float acc_x,float acc_y, float acc_z)
{
	float acc_roll = rad_to_degree*(atan2f(acc_y, acc_z));
	return acc_roll;
}

float acc_pitch(float acc_x,float acc_y, float acc_z)
{
	float acc_pitch = rad_to_degree*(atan2f(-acc_x,sqrtf((acc_y*acc_y)+(acc_z*acc_z))));
	return acc_pitch;
}


void KalmanCalculation(float State, float Uncertainty, float Input, float Measurement, float* KalmanOutput)
{
	State = State + LOOP_TIME_S*Input;
	Uncertainty = Uncertainty + LOOP_TIME_S*LOOP_TIME_S*gyro_error*gyro_error;
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
void low_pass_filter(float* current_value, float* last_filtered_value, int cnt, float gain)
{
	for(int i = 0; i < cnt;i++)
	{
		last_filtered_value[i] = last_filtered_value[i]*gain + (1-gain)*current_value[i];
	}
}

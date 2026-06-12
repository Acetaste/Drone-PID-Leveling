/*
 * Calculations.c
 *
 *  Created on: Dec 15, 2025
 *      Author: matthias Ellinger
 */




#include "calculations.h"
#include "user_constants.h"
#include <math.h>

/**
 * @brief PID controller function
 * @param Error the error to the desired Value
 * @param Prev_Error the error of the previous interation for I and D term
 * @param Prev_I the I term of the previous iteration
 * @param P the gain for the P term
 * @param I the gain for the I term
 * @param D the gain for the D term
 * @param PID_Output a pointer where the PID output and the I term are returned to
 */
void pid_equation(float Error, float Prev_Error, float Prev_I, float P, float I, float D, float* PID_Output)
{
	float Pterm = Error * P;
	float Iterm = Prev_I + (I *(Error + Prev_Error)*LOOP_TIME_S)/2;
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
/**
 * @brief function to cap the PWM before writing it to the timers
 * @param PWM the uncapped pwm value
 * @return the capped pwm value
 */
int pwm_cap(int PWM)
{
	if(PWM > PWM_CAP)
	{
		PWM = PWM_CAP;
	}
	if(PWM < 0)
	{
		PWM = 0;
	}
	return PWM;
}

/**
 * @brief function to calculate roll from accelerometer readings
 * @note uses roll,pitch, and yaw order and no compensation for instability at a pitch of ~+90°/-90°
 * @param Acc_Y the y value of the accelerometer
 * @param Acc_Z the z value of the accelerometer
 * @return the calulated roll
 */
float acc_roll(float Acc_Y, float Acc_Z)
{
	float acc_roll = RAD_TO_DEGREE*(atan2f(Acc_Y, Acc_Z));
	return acc_roll;
}

/**
 * @brief function to calculate pitch from accelerometer readings
 * @note uses roll,pitch, and yaw order
 * @param Acc_X the x value of the accelerometer
 * @param Acc_Y the y value of the accelerometer
 * @param Acc_Z the z value of the accelerometer
 * @return the calulated pitch
 */
float acc_pitch(float Acc_X,float Acc_Y, float Acc_Z)
{
	float acc_pitch = RAD_TO_DEGREE*(atan2f(-Acc_X,sqrtf((Acc_Y*Acc_Y)+(Acc_Z*Acc_Z))));
	return acc_pitch;
}

/**
 * @brief function to calculate kalman filtered angle estimation using gyroscope data and acceleremoter angle estimation
 * @param State the state of the system, equal to the last filtered angle estimation
 * @param Uncertainty the Kalman uncertainty matrix, 1x1 in this case
 * @param Input the input to the system, equal to the gyroscope data
 * @param Measurement the state measurement, equal to the accelerometer angel estimation
 * @param Kalman_Output a pointer to where the filtered output and uncertainty matrix are returned to
 */
void kalman_calculation(float State, float Uncertainty, float Input, float Measurement, float* Kalman_Output)
{
	State = State + LOOP_TIME_S*Input;
	Uncertainty = Uncertainty + LOOP_TIME_S*LOOP_TIME_S*GYRO_ERROR*GYRO_ERROR;
	float Gain = Uncertainty/(Uncertainty+ACC_ERROR*ACC_ERROR);
	State= State + Gain*(Measurement-State);
	Uncertainty = (1-Gain)*Uncertainty;
	Kalman_Output[0] = State;
	Kalman_Output[1] = Uncertainty;
}

/**
 * @brief function to covert rotation rate in the body frame to rotation rate in the fixed frame
 * @param Body_Rate a pointer to the rotation rates in the body frame in degrees
 * @param Degree_Phi the current attitude angle phi in degrees
 * @param Degree_Theta the current attitude angle theta in degrees
 * @param Degree_Output a pointer to the output of the conversion in degrees in order roll, pitch, and yaw
 */
void body_rate_to_fixed_rate(float* Body_Rate,float Degree_Phi, float Degree_Theta, float * Degree_Output)
{
	float rad_theta 	= 	Degree_Theta		* DEGREE_TO_RAD;
	float rad_phi 		= 	Degree_Phi			* DEGREE_TO_RAD;
	float rad_p			=	(*(Body_Rate+0))	* DEGREE_TO_RAD;
	float rad_q			=	(*(Body_Rate+1))	* DEGREE_TO_RAD;
	float rad_r			=	(*(Body_Rate+2))	* DEGREE_TO_RAD;
	//roll
	*(Degree_Output+0) = RAD_TO_DEGREE*(	rad_p 	+ 	rad_q * (sinf(rad_phi) * tanf(rad_theta)) 	+	rad_r * cosf(rad_phi)*tanf(rad_theta));
	//pitch
	*(Degree_Output+1) = RAD_TO_DEGREE*(				rad_q * cosf(rad_phi) 						-	rad_r * sinf(rad_phi));
	//yaw
	*(Degree_Output+2) = RAD_TO_DEGREE*(				rad_q * sinf(rad_phi) / cosf(rad_theta) 	+ 	rad_r * cosf(rad_phi)/cosf(rad_theta));
}

/**
 * @brief function to covert desired change in attitude to motor inputs
 * @param Input_Pitch a pointer to the rotation rates in the body frame in degrees
 * @param Input_Roll the current attitude angle phi in degrees
 * @param Input_Yaw the current attitude angle theta in degrees
 * @param Motor_Output a pointer to the output of the conversion in degrees in order roll, pitch, and yaw
 */
void motor_inputs(float Input_Pitch, float Input_Roll, float Input_Yaw, int* Motor_Output)
{
	float pwm_scale = 1;
	*(Motor_Output+0) = (int)(pwm_scale*	((-1) * Input_Roll		+ (+1) * Input_Pitch		+ (-1) * Input_Yaw));
	*(Motor_Output+1) = (int)(pwm_scale*	((+1) * Input_Roll		+ (+1) * Input_Pitch		+ (+1) * Input_Yaw));
	*(Motor_Output+2) = (int)(pwm_scale*	((+1) * Input_Roll		+ (-1) * Input_Pitch		+ (-1) * Input_Yaw));
	*(Motor_Output+3) = (int)(pwm_scale*	((-1) * Input_Roll		+ (-1) * Input_Pitch		+ (+1) * Input_Yaw));
}

/**
 * @brief function to cap the yaw
 * @param Yaw the uncapped yaw value
 * @return the capped yaw value
 */
float yaw_cap(float Yaw)
{
	  if(Yaw > 180)
	  {
		  Yaw -= 360;
	  }
	  if(Yaw <= -180)
	  {
		  Yaw += 360;
	  }
	  return Yaw;
}

/**
 * @brief function aplly a discrete low pass filter to Count values with a gain of Gain
 * @param Current_Value a pointer to an array of Count values of the current interation
 * @param Last_Filtered_Value a pointer to an array of Count values of the last interation the filtered value is returned here
 * @param Count the  number of values
 * @param Gain the weight between 0 and 1 placed of the last iterations values
 */
void low_pass_filter(float* Current_Value, float* Last_Filtered_Value, int Count, float Gain)
{
	for(int i = 0; i < Count;i++)
	{
		Last_Filtered_Value[i] = Last_Filtered_Value[i]*Gain + (1-Gain)*Current_Value[i];
	}
}

/*
 * calculations.h
 *
 *  Created on: Dec 15, 2025
 *      Author: matth
 */

#ifndef INC_CALCULATIONS_H_
#define INC_CALCULATIONS_H_

float acc_roll(float acc_x,float acc_y, float acc_z);
float acc_pitch(float acc_x,float acc_y, float acc_z);
void KalmanCalculation(float State, float Uncertainty, float Input, float Measurement, float* KalmanOutput);
void pid_equation(float Error, float Prev_Error, float Prev_Int, float P, float I, float D,  float* PID_Output);
int pwm_cap(int pwm);
void motor_inputs(float InputRoll, float InputPitch, float InputYaw, int* MotorInput);

#endif /* INC_CALCULATIONS_H_ */

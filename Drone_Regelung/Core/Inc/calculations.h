/*
 * calculations.h
 *
 *  Created on: Dec 15, 2025
 *      Author: matthias Ellinger
 */

#ifndef INC_CALCULATIONS_H_
#define INC_CALCULATIONS_H_



float acc_roll(float Acc_Y, float Acc_Z);
float acc_pitch(float Acc_X,float Acc_Y, float Acc_Z);
void body_rate_to_fixed_rate(float* Body_Rate,float Degree_Phi, float Degree_Theta, float * Degree_Output);
void kalman_calculation(float State, float Uncertainty, float Input, float Measurement, float* Kalman_Output);
void pid_equation(float Error, float Prev_Error, float Prev_I, float P, float I, float D,  float* PID_Output);
int pwm_cap(int PWM);
void motor_inputs(float Input_Pitch, float Input_Roll, float Input_Yaw, int* Motor_Output);
float yaw_cap(float Yaw);
void low_pass_filter(float* Current_Value, float* Last_Filtered_Value, int Count, float Gain);

#endif /* INC_CALCULATIONS_H_ */

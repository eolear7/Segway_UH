/*
 * PID.c
 *
 *  Created on: Jun 8, 2016
 *      Author: Nelson Raym Grajales
 */

#include "PID.h"

//Gloabl Variables
double PID_value; //PID Value
static double last_Error; // Previous angle Error
static double integrated_Error // Previous integration error
static double current_Speed //

void updatePID(double restAngle, double offset, double turning, double dt)
{
	double error = (restAngle - pitch);
	double p_term = seg_vals.Kp *error;
	integrated_Error += error *dt;
	// integratedError = constrain(integratedError, -1,1); //limiting the integrated values?? why?
	double i_term = (seg_vals.Ki *100.0) * integrated_Error;
	double d_term = (seg_vals.Kd / 100.0) * (error - last_Error) / dt;
	last_Error = error;
	PID_Value = p_term + i_term + dterm;

	current_speed = (current_speed+ PID_Value *0.004)*.999; // need to calculate these values
}

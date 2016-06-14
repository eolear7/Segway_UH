/*
 * Kill_Switch.c
 *
 *  Created on: Jun 13, 2016
 *      Author: Nelson Raym Grajales
 */


//#define includes
#include "kill_Switch.h"

//#define pre varibles



int kill_function()
{
	int check_kill_switch = FALSE ;

	if(/*register is on*/)
	{
		check_kill_switch = TRUE;
		UARTprintf("Kill switch has been activated");

	}
	else
		UARTprintf("Kill switch has not been activated");

	return check_kill_switch;
}

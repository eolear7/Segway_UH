/*
 * Sensor_Hub.c
 *
 *  Created on: Jun 7, 2016
 *      Author: Nelson Raym Grajales
 */

//Includes
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"
#include "drivers/rgb.h"

//--------------------------------------------Defines
// Define MPU9150 I2C Address.
//*****************************************************************************
#define MPU9150_I2C_ADDRESS     0x68

//--------------------------------------------Global Variables
// Global array for holding the color values for the RGB.
//*****************************************************************************
uint32_t g_pui32Colors[3];

// Global instance structure for the ISL29023 sensor driver.
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;

// Global instance structure for the I2C master driver.
//*****************************************************************************
tI2CMInstance g_sI2CInst;

// Global Instance structure to manage the DCM state.
//*****************************************************************************
tCompDCM g_sCompDCMInst;

// Global flags to alert main that MPU9150 I2C transaction is complete
//*****************************************************************************
volatile uint_fast8_t g_vui8I2CDoneFlag;

// Global flags to alert main that MPU9150 I2C transaction error has occurred.
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

// Global flags to alert main that MPU9150 data is ready to be retrieved.
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

// Global counter to control and slow down the rate of data to the terminal.
//*****************************************************************************
#define PRINT_SKIP_COUNT        10

//------------------------------------------other------------------------------
uint32_t g_ui32PrintSkipCounter;

// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//----------------------------------------------------Functions
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
void MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
		//
		// If the transaction succeeded set the data flag to indicate to
		// application that this transaction is complete and data may be ready.
		//
		if(ui8Status == I2CM_STATUS_SUCCESS)
		{
			g_vui8I2CDoneFlag = 1;
		}

		//
		// Store the most recent status in case it was an error condition
		//
		g_vui8ErrorFlag = ui8Status;
}




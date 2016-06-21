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

//----------------------------------------------------Functions----------------------------
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

//*****************************************************************************
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//*****************************************************************************
void IntGPIOb(void)
{
	unsigned long ulStatus;

	ulStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

	//
	// Clear all the pin interrupts that are set
	//
	GPIOIntClear(GPIO_PORTB_BASE, ulStatus);

	if(ulStatus & GPIO_PIN_2)
	{
		//
		// MPU9150 Data is ready for retrieval and processing.
		//
		MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
	}
}

//*****************************************************************************
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.

//*****************************************************************************
void MPU9150I2CIntHandler(void)
{
	//
	// Pass through to the I2CM interrupt handler provided by sensor library.
	// This is required to be at application level so that I2CMIntHandler can
	// receive the instance structure pointer as an argument.
	//
	I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//
//*****************************************************************************
void MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
	//
	// Set terminal color to red and print error status and locations
	//
	UARTprintf("\033[31;1m");
	UARTprintf("Error: %d, File: %s, Line: %d\n"
			"See I2C status definitions in sensorlib\\i2cm_drv.h\n",
			g_vui8ErrorFlag, pcFilename, ui32Line);

	//
	// Return terminal color to normal
	//
	UARTprintf("\033[0m");

	//
	// Set RGB Color to RED
	//
	g_pui32Colors[0] = 0xFFFF;
	g_pui32Colors[1] = 0;
	g_pui32Colors[2] = 0;
	RGBColorSet(g_pui32Colors);

	//
	// Increase blink rate to get attention
	//
	RGBBlinkRateSet(10.0f);

	//
	// Go to sleep wait for interventions.  A more robust application could
	// attempt corrective actions here.
	//
	while(1)
	{
		//
		// Do Nothing
		//
	}
}

//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//
//*****************************************************************************
void MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
	//
	// Put the processor to sleep while we wait for the I2C driver to
	// indicate that the transaction is complete.
	//
	while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0))
	{
		//
		// Do Nothing
		//
	}

	//
	// If an error occurred call the error handler immediately.
	//
	if(g_vui8ErrorFlag)
	{
		MPU9150AppErrorHandler(pcFilename, ui32Line);
	}

	//
	// clear the data flag for next use.
	//
	g_vui8I2CDoneFlag = 0;
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Enable UART0
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//
	// Configure GPIO Pins for UART mode.
	//
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	//
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioConfig(0, 115200, 16000000);
}

// Team Segway, Team 11.
// Emmanuel Olear, Nelson Raymond Grajales,
#include "inc/hw_types.h"
#include "driverlib/pwm.h"
#include "inc/hw_gpio.h"
#include "Kill_Switch.h"
#include "Sensor_Hub.h"
#define PWM_FREQUENCY 20000


int main(void)
{
	volatile uint32_t ui32Load;
	volatile uint32_t ui32PWMClock;
	volatile uint32_t ui8Adjust;
	ui8Adjust = 500;
	//ui8Adjust = 200;
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_2);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
	ROM_GPIOPinConfigure(GPIO_PE4_M1PWM2);
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	ROM_GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	ui32PWMClock = SysCtlClockGet() / 2;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);





	/*
	while(1)
	{
		if(ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
		{
			ui8Adjust--;
			if (ui8Adjust < 56)
			{
				ui8Adjust = 56;
			}
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);
		}
		if(ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
		{
			ui8Adjust++;
			if (ui8Adjust > 111)
			{
				ui8Adjust = 111;
			}
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);
		}
		ROM_SysCtlDelay(100000);
	}
	 */

	int_fast32_t i32IPart[16], i32FPart[16];
	uint_fast32_t ui32Idx, ui32CompDCMStarted;
	float pfData[16];
	float *pfAccel, *pfGyro, *pfMag, *pfEulers, *pfQuaternion;

	//
	// Initialize convenience pointers that clean up and clarify the code
	// meaning. We want all the data in a single contiguous array so that
	// we can make our pretty printing easier later.
	//
	pfAccel = pfData;
	pfGyro = pfData + 3;
	pfMag = pfData + 6;
	pfEulers = pfData + 9;
	pfQuaternion = pfData + 12;

	//

	//
	// Enable port B used for motion interrupt.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//
	// Initialize the UART.
	//
	ConfigureUART();

	//
	// Print the welcome message to the terminal.
	//
	UARTprintf("\033[2JMPU9150 Raw Example\n");

	//
	// Set the color to a purple approximation.
	//
	g_pui32Colors[RED] = 0x8000;
	g_pui32Colors[BLUE] = 0x8000;
	g_pui32Colors[GREEN] = 0x0000;

	//
	// Initialize RGB driver.
	//
	RGBInit(0);
	RGBColorSet(g_pui32Colors);
	RGBIntensitySet(0.5f);
	RGBEnable();

	//
	// The I2C3 peripheral must be enabled before use.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	//
	// Configure the pin muxing for I2C3 functions on port D0 and D1.
	//
	ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

	//
	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	//
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

	//
	// Configure and Enable the GPIO interrupt. Used for INT signal from the
	// MPU9150
	//
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
	ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
	ROM_IntEnable(INT_GPIOB);

	//
	// Keep only some parts of the systems running while in sleep mode.
	// GPIOB is for the MPU9150 interrupt pin.
	// UART0 is the virtual serial port
	// TIMER0, TIMER1 and WTIMER5 are used by the RGB driver
	// I2C3 is the I2C interface to the ISL29023
	//
	ROM_SysCtlPeripheralClockGating(true);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C3);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5);

	//
	// Enable interrupts to the processor.
	//
	ROM_IntMasterEnable();

	//
	// Initialize I2C3 peripheral.
	//
	I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
			ROM_SysCtlClockGet());

	//
	// Initialize the MPU9150 Driver.
	//
	MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
			MPU9150AppCallback, &g_sMPU9150Inst);

	//
	// Wait for transaction to complete
	//
	MPU9150AppI2CWait(__FILE__, __LINE__);

	//
	// Write application specifice sensor configuration such as filter settings
	// and sensor range settings.
	//
	g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_94_98;
	g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
	g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
			MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
			MPU9150AppCallback, &g_sMPU9150Inst);

	//
	// Wait for transaction to complete
	//
	MPU9150AppI2CWait(__FILE__, __LINE__);

	//
	// Configure the data ready interrupt pin output of the MPU9150.
	//
	g_sMPU9150Inst.pui8Data[0] = MPU9150_INT_PIN_CFG_INT_LEVEL |
			MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
			MPU9150_INT_PIN_CFG_LATCH_INT_EN;
	g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
			g_sMPU9150Inst.pui8Data, 2, MPU9150AppCallback,
			&g_sMPU9150Inst);

	//
	// Wait for transaction to complete
	//
	MPU9150AppI2CWait(__FILE__, __LINE__);

	//
	// Initialize the DCM system. 50 hz sample rate.
	// accel weight = .2, gyro weight = .8, mag weight = .2
	//
	CompDCMInit(&g_sCompDCMInst, 1.0f / 50.0f, 0.2f, 0.6f, 0.2f);

	UARTprintf("\033[2J\033[H");
	UARTprintf("MPU9150 9-Axis Simple Data Application Example\n\n");
	UARTprintf("\033[20GX\033[31G|\033[43GY\033[54G|\033[66GZ\n\n");
	// UARTprintf("Accel\033[8G|\033[31G|\033[54G|\n\n");
	//  UARTprintf("Gyro\033[8G|\033[31G|\033[54G|\n\n");
	// UARTprintf("Mag\033[8G|\033[31G|\033[54G|\n\n");
	UARTprintf("\n\033[20GRoll\033[31G|\033[43GPitch\033[54G|\033[66GYaw\n\n");
	UARTprintf("Eulers\033[8G|\033[31G|\033[54G|\n\n");

	//    UARTprintf("\n\033[17GQ1\033[26G|\033[35GQ2\033[44G|\033[53GQ3\033[62G|"
	//             "\033[71GQ4\n\n");
	//  UARTprintf("Q\033[8G|\033[26G|\033[44G|\033[62G|\n\n");

	//
	// Enable blinking indicates config finished successfully
	//
	RGBBlinkRateSet(1.0f);

	ui32CompDCMStarted = 0;

	while(1)
	{
		//
		// Go to sleep mode while waiting for data ready.
		//
		while(!g_vui8I2CDoneFlag)
		{
			//ROM_SysCtlSleep();
		}

		//
		// Clear the flag
		//
		g_vui8I2CDoneFlag = 0;

		//
		// Get floating point version of the Accel Data in m/s^2.
		//
		MPU9150DataAccelGetFloat(&g_sMPU9150Inst, pfAccel, pfAccel + 1,
				pfAccel + 2);

		//
		// Get floating point version of angular velocities in rad/sec
		//
		MPU9150DataGyroGetFloat(&g_sMPU9150Inst, pfGyro, pfGyro + 1,
				pfGyro + 2);

		//
		// Get floating point version of magnetic fields strength in tesla
		//
		MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, pfMag, pfMag + 1,
				pfMag + 2);

		//
		// Check if this is our first data ever.
		//




		if(ui32CompDCMStarted == 0)
		{
			//
			// Set flag indicating that DCM is started.
			// Perform the seeding of the DCM with the first data set.
			//
			ui32CompDCMStarted = 1;
			CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
					pfMag[2]);
			CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
					pfAccel[2]);
			CompDCMGyroUpdate(&g_sCompDCMInst, pfGyro[0], pfGyro[1],
					pfGyro[2]);
			CompDCMStart(&g_sCompDCMInst);
		}
		else
		{
			//
			// DCM Is already started.  Perform the incremental update.
			//
			CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
					pfMag[2]);
			CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
					pfAccel[2]);
			CompDCMGyroUpdate(&g_sCompDCMInst, -pfGyro[0], -pfGyro[1],
					-pfGyro[2]);
			CompDCMUpdate(&g_sCompDCMInst);
		}

		//
		// Increment the skip counter.  Skip counter is used so we do not
		// overflow the UART with data.
		//
		g_ui32PrintSkipCounter++;
		if(g_ui32PrintSkipCounter >= PRINT_SKIP_COUNT)
		{
			//
			// Reset skip counter.
			//
			g_ui32PrintSkipCounter = 0;

			//
			// Get Euler data. (Roll Pitch Yaw)
			//
			CompDCMComputeEulers(&g_sCompDCMInst, pfEulers, pfEulers + 1,
					pfEulers + 2);

			//
			// Get Quaternions.
			//
			CompDCMComputeQuaternion(&g_sCompDCMInst, pfQuaternion);

			//
			// convert mag data to micro-tesla for better human interpretation.
			//
			pfMag[0] *= 1e6;
			pfMag[1] *= 1e6;
			pfMag[2] *= 1e6;

			//
			// Convert Eulers to degrees. 180/PI = 57.29...
			// Convert Yaw to 0 to 360 to approximate compass headings.
			//
			pfEulers[0] *= 57.295779513082320876798154814105f;
			pfEulers[1] *= 57.295779513082320876798154814105f;
			pfEulers[2] *= 57.295779513082320876798154814105f;
			if(pfEulers[2] < 0)
			{
				pfEulers[2] += 360.0f;
			}

			//
			// Now drop back to using the data as a single array for the
			// purpose of decomposing the float into a integer part and a
			// fraction (decimal) part.
			//
			for(ui32Idx = 0; ui32Idx < 16; ui32Idx++)
			{
				//
				// Conver float value to a integer truncating the decimal part.
				//
				i32IPart[ui32Idx] = (int32_t) pfData[ui32Idx];

				//
				// Multiply by 1000 to preserve first three decimal values.
				// Truncates at the 3rd decimal place.
				//
				i32FPart[ui32Idx] = (int32_t) (pfData[ui32Idx] * 1000.0f);

				//
				// Subtract off the integer part from this newly formed decimal
				// part.
				//
				i32FPart[ui32Idx] = i32FPart[ui32Idx] -
						(i32IPart[ui32Idx] * 1000);

				//
				// make the decimal part a positive number for display.
				//
				if(i32FPart[ui32Idx] < 0)
				{
					i32FPart[ui32Idx] *= -1;
				}
			}

			//
			/*
			  // Print the acceleration numbers in the table.
	            //
	            UARTprintf("\033[5;17H%3d.%03d", i32IPart[0], i32FPart[0]);
	            UARTprintf("\033[5;40H%3d.%03d", i32IPart[1], i32FPart[1]);
	            UARTprintf("\033[5;63H%3d.%03d", i32IPart[2], i32FPart[2]);


	            //
	            // Print the angular velocities in the table.
	            //
	            UARTprintf("\033[7;17H%3d.%03d", i32IPart[3], i32FPart[3]);
	            UARTprintf("\033[7;40H%3d.%03d", i32IPart[4], i32FPart[4]);
	            UARTprintf("\033[7;63H%3d.%03d", i32IPart[5], i32FPart[5]);

	            //
	            // Print the magnetic data in the table.
	            //
	            UARTprintf("\033[9;17H%3d.%03d", i32IPart[6], i32FPart[6]);
	            UARTprintf("\033[9;40H%3d.%03d", i32IPart[7], i32FPart[7]);
	            UARTprintf("\033[9;63H%3d.%03d", i32IPart[8], i32FPart[8]);
			 */
			//
			// Print the Eulers in a table.

			//
			UARTprintf("\033[14;17H%3d.%03d", i32IPart[9], i32FPart[9]);
			UARTprintf("\033[14;40H%3d.%03d", i32IPart[10], i32FPart[10]);
			UARTprintf("\033[14;63H%3d.%03d", i32IPart[11], i32FPart[11]);


			/*
			if(ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
					{
						ui8Adjust--;
						if (ui8Adjust < 56)
						{
							ui8Adjust = 56;
						}
						ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);
					}
					if(ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
					{
						ui8Adjust++;
						if (ui8Adjust > 111)
						{
							ui8Adjust = 111;
						}
						ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);
					}
					ROM_SysCtlDelay(1000);
		}
			 */
			/*
			//
	            // Print the quaternions in a table format.
	            //
	            UARTprintf("\033[19;14H%3d.%03d", i32IPart[12], i32FPart[12]);
	            UARTprintf("\033[19;32H%3d.%03d", i32IPart[13], i32FPart[13]);
	            UARTprintf("\033[19;50H%3d.%03d", i32IPart[14], i32FPart[14]);
	            UARTprintf("\033[19;68H%3d.%03d", i32IPart[15], i32FPart[15]);
			 */

			while(kill_function())
			{
			if(i32IPart[9] > 20)
			{
				ui8Adjust--;
				if (ui8Adjust < 200)
				{
					ui8Adjust = 200;
				}
				ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);

			}
			if(i32IPart[9] < -5)
			{
				ui8Adjust++;
				if (ui8Adjust > 900)
				{
					ui8Adjust = 900;
				}
				ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);

			}
			//ROM_SysCtlDelay(50000);
		}

		if(i32IPart[9] > 20)
		{
			ui8Adjust--;
			if (ui8Adjust < 200)
			{
				ui8Adjust = 200;
			}
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);

		}
		if(i32IPart[9] < -5)
		{
			ui8Adjust++;
			if (ui8Adjust > 900)
			{
				ui8Adjust = 900;
			}
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);

		}
		}
		//ROM_SysCtlDelay(50000);
/*
		if(i32IPart[9] > 20)
		{
			ui8Adjust--;
			if (ui8Adjust < 56)
			{
				ui8Adjust = 56;
			}
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);

		}
		if(i32IPart[9] < -5)
		{
			ui8Adjust++;
			if (ui8Adjust > 700)
			{
				ui8Adjust = 700;
			}
			ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui8Adjust * ui32Load / 1000);

		} */
	}





}




/*
 * Sensor_Hub.h
 *
 *  Created on: Jun 7, 2016
 *      Author: Nelson Raym Grajales
 */
#ifndef SENSOR_HUB_H_
#define SENSOR_HUB_H_


void MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status);
void IntGPIOb(void);
void MPU9150I2CIntHandler(void);
void MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line);
void MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line);
void MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line);
void ConfigureUART(void);







#endif /* SENSOR_HUB_H_ */

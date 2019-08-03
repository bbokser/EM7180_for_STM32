/*
 * EM7180_MPU9250_BMP280.c
 *
 *  Created on: May 1, 2019
 *      Author: Ben
 */

/* EM7180_MPU9250_BMP280_t3 Basic Example Code
 by: Kris Winer
 date: September 11, 2015
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 The EM7180 SENtral sensor hub is not a motion sensor, but rather takes raw sensor data from a variety of motion sensors,
 in this case the MPU9250 (with embedded MPU9250 + AK8963C), and does sensor fusion with quaternions as its output. The SENtral loads firmware from the
 on-board M24512DRC 512 kbit EEPROM upon startup, configures and manages the sensors on its dedicated master I2C bus,
 and outputs scaled sensor data (accelerations, rotation rates, and magnetic fields) as well as quaternions and
 heading/pitch/roll, if selected.

 This sketch demonstrates basic EM7180 SENtral functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 */

#include "EM7180_MPU9250_BMP280.h"
#include <stdio.h>
#include "i2c.h"
#include "cmsis_os.h"

#include <stdbool.h>
#include "usart.h"
#include <string.h>

em7180_struct em7180;

extern char buffer[];
extern int len;
//char buffer[51];
//int len;

// Specify BMP280 configuration
	uint8_t Posr = P_OSR_16;
	uint8_t Tosr = T_OSR_02;
	uint8_t Mode = normal;
	uint8_t IIRFilter = BW0_042ODR;
	uint8_t SBy = t_62_5ms;     // set pressure and temperature output data rate
	// t_fine carries fine temperature as global value for BMP280
	int32_t t_fine;

	//
	// Specify sensor full scale
	uint8_t Gscale = GFS_250DPS;
	uint8_t Ascale = AFS_2G;
	uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
	uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

	// BMP280 compensation parameters
	uint16_t dig_T1, dig_P1;
	int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	double Temperature, Pressure; // stores BMP280 pressures sensor pressure and temperature
	int32_t rawPress, rawTemp;   // pressure and temperature raw count output for BMP280

	// MPU9250 variables
	int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
	int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
	int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
	float Quat[4] = {0, 0, 0, 0}; // quaternion data register
	float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
	float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
	int16_t tempCount, rawPressure, rawTemperature;   // pressure, temperature raw count output
	float temperature, pressure, altitude; // Stores the MPU9250 internal chip temperature in degrees Celsius
	float SelfTest[6];            // holds results of gyro and accelerometer self test

	uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
	float pitch, yaw, roll, Yaw, Pitch, Roll;
	float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
	uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
	uint32_t Now = 0;                         // used to calculate integration interval
	uint8_t param[4];                         // used for param transfer
	uint16_t EM7180_mag_fs, EM7180_acc_fs, EM7180_gyro_fs; // EM7180 sensor full scale ranges

	float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values

void EM7180_setup()
{
	// Read SENtral device information
	  uint16_t ROM1;
	  HAL_I2C_Mem_Read(&hi2c1,EM7180_ADDRESS, EM7180_ROMVersion1, 1, (uint8_t*)&ROM1, 1, 10);
	  uint16_t ROM2;
	  HAL_I2C_Mem_Read(&hi2c1,EM7180_ADDRESS, EM7180_ROMVersion2, 1, (uint8_t*)&ROM2, 1, 10);
	  sprintf(buffer, "EM7180 ROM Version: 0x%x  \r\n Should be: 0xE609 \r\n", ROM2);
	  len=strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);

	  uint16_t RAM1;
	  HAL_I2C_Mem_Read(&hi2c1,EM7180_ADDRESS, EM7180_RAMVersion1, 1, (uint8_t*)&RAM1, 1, 10);
	  uint16_t RAM2;
	  HAL_I2C_Mem_Read(&hi2c1,EM7180_ADDRESS, EM7180_RAMVersion2, 1, (uint8_t*)&RAM2, 1, 10);
	  sprintf(buffer, "EM7180 RAM Version: 0x%x \r\n %u", RAM1, RAM2);
	  len=strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);

	  uint8_t PID;
	  HAL_I2C_Mem_Read(&hi2c1,EM7180_ADDRESS, EM7180_ProductID, 1, &PID, 1, 10);
	  sprintf(buffer, "EM7180 ProductID: 0x%x Should be: 0x80 \r\n", PID);
	  len=strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);

	  uint8_t RID;
	  HAL_I2C_Mem_Read(&hi2c1,EM7180_ADDRESS, EM7180_RevisionID, 1, &RID, 1, 10);
	  sprintf(buffer, "EM7180 RevisionID: 0x%x Should be: 0x02 \r\n", RID);
	  len=strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);

	  osDelay(2000); // give some time to read the screen

	  // Check which sensors can be detected by the EM7180
	  uint8_t featureflag;
	  HAL_I2C_Mem_Read(&hi2c1,EM7180_ADDRESS, EM7180_FeatureFlags,1,&featureflag,1,10);
	    if(featureflag & 0x01)  {
	    	sprintf(buffer, "A barometer is installed \r\n");
	    	len=strlen(buffer);
	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    }
	    if(featureflag & 0x02)  {
	    	sprintf(buffer, "A humidity sensor is installed \r\n");
	    	len=strlen(buffer);
	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    }
	    if(featureflag & 0x04)  {
	    	sprintf(buffer, "A temperature sensor is installed \r\n");
	    	len=strlen(buffer);
	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    }
	    if(featureflag & 0x08)  {
	    	sprintf(buffer, "A custom sensor is installed \r\n");
	    	len=strlen(buffer);
	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    }
	    if(featureflag & 0x10)  {
	    	sprintf(buffer, "A second custom sensor is installed \r\n");
	    	len=strlen(buffer);
	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    }
	    if(featureflag & 0x20)  {
	    	sprintf(buffer, "A third custom sensor is installed \r\n");
	    	len=strlen(buffer);
	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    }
	  osDelay(1000); // give some time to read the screen

	  // Check SENtral status, make sure EEPROM upload of firmware was accomplished
	  uint8_t STAT;
	  HAL_I2C_Mem_Read(&hi2c1,EM7180_ADDRESS, EM7180_SentralStatus,1,&STAT,1,10);
	  STAT= STAT & 0x01;
	    if(STAT & 0x01)  {
	    	sprintf(buffer,"EEPROM detected on the sensor bus! \r\n");
	    	len=strlen(buffer);
	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    }

	    if(STAT & 0x02)  {
	    	sprintf(buffer,"EEPROM uploaded config file! \r\n");
	    	len=strlen(buffer);
	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    }
	    if(STAT & 0x04)  {
	    	sprintf(buffer,"EEPROM CRC incorrect! \r\n");
	    	len=strlen(buffer);
	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    }

	    if(STAT & 0x08)  {
	    	sprintf(buffer,"EM7180 in initialized state! \r\n");
	    	len=strlen(buffer);
	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    }
	    if(STAT & 0x10)  {
	    	sprintf(buffer,"No EEPROM detected! \r\n");
	    	len=strlen(buffer);
	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    }
	  int count = 0;
	  uint8_t tempvar = 0x01;
	  uint8_t zero = 0x00;
	  while(!STAT) {
		HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ResetRequest,1, &tempvar,1,10);
	    osDelay(500);
	    count++;
	    HAL_I2C_Mem_Read(&hi2c1,EM7180_ADDRESS, EM7180_SentralStatus,1,&STAT,1,10);
	    STAT = STAT & 0x01;
	    if(STAT & 0x01)  {
	    	    	sprintf(buffer,"EEPROM detected on the sensor bus! \r\n");
	    	    	len=strlen(buffer);
	    	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    	    }

	    	    if(STAT & 0x02)  {
	    	    	sprintf(buffer,"EEPROM uploaded config file! \r\n");
	    	    	len=strlen(buffer);
	    	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    	    }
	    	    if(STAT & 0x04)  {
	    	    	sprintf(buffer,"EEPROM CRC incorrect! \r\n");
	    	    	len=strlen(buffer);
	    	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    	    }

	    	    if(STAT & 0x08)  {
	    	    	sprintf(buffer,"EM7180 in initialized state! \r\n");
	    	    	len=strlen(buffer);
	    	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    	    }
	    	    if(STAT & 0x10)  {
	    	    	sprintf(buffer,"No EEPROM detected! \r\n");
	    	    	len=strlen(buffer);
	    	    	HAL_UART_Transmit(&huart2, buffer, len, 10);
	    	    }
	    if(count > 10) break;
	  }

	   HAL_I2C_Mem_Read(&hi2c1,EM7180_ADDRESS,EM7180_SentralStatus,1,&STAT,1,10);
	   if(!(STAT & 0x04))  sprintf(buffer,"EEPROM upload successful! \r\n");
	   len=strlen(buffer);
	   HAL_UART_Transmit(&huart2, buffer, len, 10);
	   osDelay(1000); // give some time to read the screen

	// Set up the SENtral as sensor bus in normal operating mode
	// Enter EM7180 initialized state

	HAL_I2C_Mem_Write(&hi2c1,  EM7180_ADDRESS, EM7180_HostControl, 1, &zero, 1, 10); // set SENtral in initialized state to configure registers
	HAL_I2C_Mem_Write(&hi2c1,  EM7180_ADDRESS, EM7180_PassThruControl, 1, &zero, 1, 10); // make sure pass through mode is off
	tempvar = 0x01;
	HAL_I2C_Mem_Write(&hi2c1,  EM7180_ADDRESS, EM7180_HostControl, 1, &tempvar, 1, 10); // Force initialize
	HAL_I2C_Mem_Write(&hi2c1,  EM7180_ADDRESS, EM7180_HostControl, 1, &zero, 1, 10); // set SENtral in initialized state to configure registers

	//Setup LPF bandwidth (BEFORE setting ODR's)
	tempvar = 0x03;
	HAL_I2C_Mem_Write(&hi2c1,  EM7180_ADDRESS, EM7180_ACC_LPF_BW, 1, &tempvar, 1, 10); // 41Hz
	HAL_I2C_Mem_Write(&hi2c1,  EM7180_ADDRESS, EM7180_GYRO_LPF_BW, 1, &tempvar, 1, 10); // 41Hz
	// Set accel/gyro/mage desired ODR rates
	tempvar = 0x02;
	HAL_I2C_Mem_Write(&hi2c1,  EM7180_ADDRESS, EM7180_QRateDivisor, 1, &tempvar, 1, 10); // 100 Hz
	tempvar = 0x64;
	HAL_I2C_Mem_Write(&hi2c1,  EM7180_ADDRESS, EM7180_MagRate, 1, &tempvar, 1, 10); // 100 Hz
	tempvar = 0x14;
	HAL_I2C_Mem_Write(&hi2c1,  EM7180_ADDRESS, EM7180_AccelRate, 1, &tempvar, 1, 10); // 200/10 Hz
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_GyroRate, 1, &tempvar, 1, 10); // 200/10 Hz
	uint8_t BaroEnable = 0x80 | 0x32;
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_BaroRate, 1, &BaroEnable, 1, 10);  // set enable bit and set Baro rate to 25 Hz
	// HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_TempRate, 0x19);  // set enable bit and set rate to 25 Hz

	// Configure operating mode
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, &zero, 1, 10); // read scale sensor data
	// Enable interrupt to host upon certain events
	// choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
	// new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
	tempvar = 0x07;
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_EnableEvents, 1, &tempvar, 1, 10);
	// Enable EM7180 run mode
	tempvar = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_HostControl, 1, &tempvar, 1, 10); // set SENtral in normal run mode
	osDelay(100);

	// EM7180 parameter adjustments

	// Read sensor default FS values from parameter space
	tempvar = 0x4A;
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, &tempvar, 1, 10); // Request to read parameter 74
	tempvar = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, &tempvar, 1, 10); // Request parameter transfer process
	uint8_t param_xfer;
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &param_xfer, 1, 10);

	while(!(param_xfer==0x4A)) {
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &param_xfer, 1, 10);
	sprintf(buffer, "param_xfer = %u \r\n", param_xfer);
	len=strlen(buffer);
	HAL_UART_Transmit(&huart2, buffer, len, 10);
	}

	osDelay(3000);

	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte0, 1, &param[0], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte1, 1, &param[1], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte2, 1, &param[2], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte3, 1, &param[3], 1, 10);
	EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
	EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);

	sprintf(buffer, "Mag Default Range: +/- %u uT \r\n", EM7180_mag_fs);
	len=strlen(buffer);
	HAL_UART_Transmit(&huart2, buffer, len, 10);
	sprintf(buffer, "Acc Default Range: +/- %u g \r\n", EM7180_acc_fs);
	len=strlen(buffer);
	HAL_UART_Transmit(&huart2, buffer, len, 10);

	tempvar = 0x4B;
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, &tempvar, 1, 10); // Request to read  parameter 75
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &param_xfer, 1, 10);
	while(!(param_xfer==0x4B)) {
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &param_xfer, 1, 10);
	}
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte0, 1, &param[0], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte1, 1, &param[1], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte2, 1, &param[2], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte3, 1, &param[3], 1, 10);
	EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);

	sprintf(buffer, "Gyro Default Range: +/- %u dps \r\n", EM7180_gyro_fs);
	len=strlen(buffer);
	HAL_UART_Transmit(&huart2, buffer, len, 10);

	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, &zero, 1, 10); //End parameter transfer
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, &zero, 1, 10); // re-enable algorithm

	//Disable stillness mode
	EM7180_set_integer_param (0x49, 0x00);

	//Write desired sensor full scale ranges to the EM7180
	EM7180_set_mag_acc_FS (0x3E8, 0x08); // 1000 uT, 8 g
	EM7180_set_gyro_FS (0x7D0); // 2000 dps

	// Read sensor new FS values from parameter space
	tempvar = 0x4A;
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, &tempvar, 1, 10); // Request to read  parameter 74
	tempvar = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, &tempvar, 1, 10); // Request parameter transfer process
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &param_xfer, 1, 10);
	while(!(param_xfer==0x4A)) {
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &param_xfer, 1, 10);
	}
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte0, 1, &param[0], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte1, 1, &param[1], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte2, 1, &param[2], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte3, 1, &param[3], 1, 10);
	EM7180_mag_fs = ((int16_t)(param[1]<<8) | param[0]);
	EM7180_acc_fs = ((int16_t)(param[3]<<8) | param[2]);

	sprintf(buffer, "Mag New Range: +/- %u uT \r\n", EM7180_mag_fs);
	len=strlen(buffer);
	HAL_UART_Transmit(&huart2, buffer, len, 10);
	sprintf(buffer, "Acc New Range: +/- %u g \r\n", EM7180_acc_fs);
	len=strlen(buffer);
	HAL_UART_Transmit(&huart2, buffer, len, 10);

	tempvar = 0x4B;
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, &tempvar, 1, 10); // Request to read  parameter 75
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &param_xfer, 1, 10);
	while(!(param_xfer==0x4B)) {
	  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &param_xfer, 1, 10);
	}
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte0, 1, &param[0], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte1, 1, &param[1], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte2, 1, &param[2], 1, 10);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SavedParamByte3, 1, &param[3], 1, 10);
	EM7180_gyro_fs = ((int16_t)(param[1]<<8) | param[0]);

	sprintf(buffer, "Gyro New Range: +/- %u dps \r\n", EM7180_gyro_fs);
	len=strlen(buffer);
	HAL_UART_Transmit(&huart2, buffer, len, 10);

	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, &zero, 1, 10); //End parameter transfer
	HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, &zero, 1, 10); // re-enable algorithm


	// Read EM7180 status
	uint8_t runStatus;
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_RunStatus, 1, &runStatus, 1, 10);
	if(runStatus & 0x01){
	  sprintf(buffer, "EM7180 run status = normal mode \r\n");
	  len=strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	uint8_t algoStatus;
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmStatus, 1, &algoStatus, 1, 10);
	if(algoStatus & 0x01) {
		sprintf(buffer, "EM7180 standby status \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(algoStatus & 0x02) {
		sprintf(buffer, "EM7180 algorithm slow \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(algoStatus & 0x04) {
		sprintf(buffer, "EM7180 in stillness mode \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(algoStatus & 0x08) {
		sprintf(buffer, "EM7180 mag calibration completed \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(algoStatus & 0x10) {
		sprintf(buffer, "EM7180 magnetic anomaly detected \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(algoStatus & 0x20) {
		sprintf(buffer, "EM7180 unreliable sensor data \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	uint8_t passthruStatus;
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_PassThruStatus, 1, &passthruStatus, 1, 10);
	if(passthruStatus & 0x01) {
		sprintf(buffer, " EM7180 in passthru mode! \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	uint8_t eventStatus;
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_EventStatus, 1, &eventStatus, 1, 10);
	if(eventStatus & 0x01) {
		sprintf(buffer, "EM7180 CPU reset \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(eventStatus & 0x02) {
		sprintf(buffer, "EM7180 Error \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(eventStatus & 0x04) {
		sprintf(buffer, "EM7180 new quaternion result \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(eventStatus & 0x08) {
		sprintf(buffer, "EM7180 new mag result \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(eventStatus & 0x10) {
		sprintf(buffer, "EM7180 new accel result \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(eventStatus & 0x20) {
		sprintf(buffer, "EM7180 new gyro result \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	osDelay(1000); // give some time to read the screen

	// Check sensor status
	uint8_t sensorStatus;
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_SensorStatus, 1, &sensorStatus, 1, 10);
	sprintf(buffer, "EM7180 sensor status = %u \r\n", sensorStatus);
	len=strlen(buffer);
	HAL_UART_Transmit(&huart2, buffer, len, 10);
	if(sensorStatus & 0x01) {
		sprintf(buffer, "Magnetometer not acknowledging! \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(sensorStatus & 0x02) {
		sprintf(buffer, "Accelerometer not acknowledging! \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(sensorStatus & 0x04) {
		sprintf(buffer, "Gyro not acknowledging! \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(sensorStatus & 0x10) {
		sprintf(buffer, "Magnetometer ID not recognized! \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(sensorStatus & 0x20) {
		sprintf(buffer, "Accelerometer ID not recognized! \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	if(sensorStatus & 0x40) {
		sprintf(buffer, "Gyro ID not recognized! \r\n");
		len=strlen(buffer);
		HAL_UART_Transmit(&huart2, buffer, len, 10);
	}
	uint8_t Hz;
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ActualMagRate, 1, &Hz, 1, 10);
	sprintf(buffer, "Actual MagRate = %u Hz", Hz);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ActualAccelRate, 1, &Hz, 1, 10);
	sprintf(buffer, "Actual AccelRate = %u Hz", Hz);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ActualGyroRate, 1, &Hz, 1, 10);
	sprintf(buffer, "Actual GyroRate = %u Hz", Hz);
	HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ActualBaroRate, 1, &Hz, 1, 10);
	sprintf(buffer, "Actual BaroRate = %u Hz", Hz);
	//  sprintf(buffer, "Actual TempRate = "); sprintf(buffer, HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ActualTempRate)); sprintf(buffer, " Hz");

	osDelay(1000); // give some time to read the screen
}


void read_EM7180(float * euler_angle, float * gyr)
{
  // Check event status register, way to check data ready by polling rather than interrupt
  uint8_t eventStatus;
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_EventStatus, 1, &eventStatus, 1, 10); // reading clears the register
  // Check for errors
  if(eventStatus & 0x02) { // error detected, what is it?
  uint8_t errorStatus;
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ErrorRegister, 1, &errorStatus, 1, 10);
  if(errorStatus != 0x00) { // non-zero value indicates error, what is it?
  if(errorStatus == 0x11) {
	  sprintf(buffer, "EM7180 sensor status = Magnetometer failure! /r/n");
	  len = strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);
  }
  if(errorStatus == 0x12) {
	  sprintf(buffer, "EM7180 sensor status = Accelerometer failure! /r/n");
	  len = strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);
  }
  if(errorStatus == 0x14) {
	  sprintf(buffer, "EM7180 sensor status = Gyro failure! /r/n");
	  len = strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);
  }
  if(errorStatus == 0x21) {
	  sprintf(buffer, "EM7180 sensor status = Mag init failure! /r/n");
	  len = strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);
  }
  if(errorStatus == 0x22) {
	  sprintf(buffer, "EM7180 sensor status = Acc init failure! /r/n");
	  len = strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);
  }
  if(errorStatus == 0x24) {
	  sprintf(buffer, "EM7180 sensor status = Gyro init failure! /r/n");
	  len = strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);
  }
  if(errorStatus == 0x30) {
	  sprintf(buffer, "EM7180 sensor status = Math error! /r/n");
	  len = strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);
  }
  if(errorStatus == 0x80) {
	  sprintf(buffer, "EM7180 sensor status = Invalid sample rate! /r/n");
	  len = strlen(buffer);
	  HAL_UART_Transmit(&huart2, buffer, len, 10);
  }

  	  }
  }


 // if no errors, see if new data is ready
  if(eventStatus & 0x10) { // new acceleration data available
     readSENtralAccelData(accelCount);

    // Now we'll calculate the acceleration value into actual g's
    ax = (float)accelCount[0]*0.000488;  // get actual g value
    ay = (float)accelCount[1]*0.000488;
    az = (float)accelCount[2]*0.000488;

  }

   if(eventStatus & 0x20) { // new gyro data available
    readSENtralGyroData(gyroCount);

    // Now we'll calculate the gyro value into actual dps's
    gx = (float)gyroCount[0]*0.153;  // get actual dps value
    gy = (float)gyroCount[1]*0.153;
    gz = (float)gyroCount[2]*0.153;
   }

  if(eventStatus & 0x08) { // new mag data available
    readSENtralMagData(magCount);

    // Now we'll calculate the mag value into actual G's
    mx = (float)magCount[0]*0.305176;  // get actual G value
    my = (float)magCount[1]*0.305176;
    mz = (float)magCount[2]*0.305176;
   }

  if(eventStatus & 0x04) { // new quaternion data available
    readSENtralQuatData(Quat);
    }

 // get BMP280 pressure
   if(eventStatus & 0x40) { // new baro data available
 //   sprintf(buffer, "new Baro data!");
    rawPressure = readSENtralBaroData();
    pressure = (float)rawPressure*0.01f +1013.25f; // pressure in mBar

    // get BMP280 temperature
    rawTemperature = readSENtralTempData();
    temperature = (float) rawTemperature*0.01;  // temperature in degrees C
   }

  // Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // We will assume that +y accel/gyro is North, then x accel/gyro is East. So if we want te quaternions properly aligned
  // we need to feed into the madgwick function Ay, Ax, -Az, Gy, Gx, -Gz, Mx, My, and Mz. But because gravity is by convention
  // positive down, we need to invert the accel data, so we pass -Ay, -Ax, Az, Gy, Gx, -Gz, Mx, My, and Mz into the Madgwick
  // function to get North along the accel +y-axis, East along the accel +x-axis, and Down along the accel -z-axis.
  // This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s

// tempCount = readTempData();  // Read the gyro adc values
//    temperature = ((float) tempCount) / 333.87 + 21.0; // Gyro chip temperature in degrees Centigrade
   // Print temperature in degrees Centigrade
//    sprintf(buffer, "Gyro temperature is ");  sprintf(buffer, temperature, 1);  sprintf(buffer, " degrees C"); // Print T values to tenths of s degree C

  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth.
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    //Software AHRS:

    //Hardware AHRS:
    Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);
    Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
    Roll  = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
    Pitch *= 180.0f / M_PI;
    Yaw   *= 180.0f / M_PI;
    Yaw   += -14.4f; // Declination at Framingham, Massachusetts is negative 14 degrees 24 minutes 2019-05-05
    if(Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
    Roll  *= 180.0f / M_PI;

    // Or define output variable according to the Android system, where heading (0 to 360) is defined by the angle between the y-axis
    // and True North, pitch is rotation about the x-axis (-180 to +180), and roll is rotation about the y-axis (-90 to +90)
    // In this systen, the z-axis is pointing away from Earth, the +y-axis is at the "top" of the device (cellphone) and the +x-axis
    // points toward the right of the device.
    //
    euler_angle[0] = Yaw;
    euler_angle[1] = Roll;
	euler_angle[2] = Pitch;

	gyr[0] = gx;
	gyr[1] = gy;
	gyr[2] = gz;

 //     sprintf(buffer, yaw); sprintf(buffer, ",");sprintf(buffer, pitch); sprintf(buffer, ",");sprintf(buffer, roll); sprintf(buffer, ",");
 //     sprintf(buffer, Yaw); sprintf(buffer, ",");sprintf(buffer, Pitch); sprintf(buffer, ",");sprintf(buffer, Roll);

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

float uint32_reg_to_float (uint8_t *buf)
{
  union {
    uint32_t ui32;
    float f;
  } u;

  u.ui32 =     (((uint32_t)buf[0]) +
               (((uint32_t)buf[1]) <<  8) +
               (((uint32_t)buf[2]) << 16) +
               (((uint32_t)buf[3]) << 24));
  return u.f;
}

void float_to_bytes (float param_val, uint8_t *buf) {
  union {
    float f;
    uint8_t comp[sizeof(float)];
  } u;
  u.f = param_val;
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = u.comp[i];
  }
  //Convert to LITTLE ENDIAN
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = buf[(sizeof(float)-1) - i];
  }
}

void EM7180_set_gyro_FS (uint16_t gyro_fs) {
  uint8_t bytes[4], STAT, tempvar;
  bytes[0] = gyro_fs & (0xFF);
  bytes[1] = (gyro_fs >> 8) & (0xFF);
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte0, 1, &bytes[0], 1, 10); //Gyro LSB
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte1, 1, &bytes[1], 1, 10); //Gyro MSB
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte2, 1, &bytes[2], 1, 10); //Unused
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte3, 1, &bytes[3], 1, 10); //Unused
  tempvar = 0xCB;
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, &tempvar, 1, 10); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
  tempvar = 0x80;
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, &tempvar, 1, 10); //Request parameter transfer procedure
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &STAT, 1, 10); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==0xCB)) {
	  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &STAT, 1, 10);
  }
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, 0x00, 1, 10); //Parameter request = 0 to end parameter transfer process
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, 0x00, 1, 10); // Re-start algorithm
}

void EM7180_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs) {
  uint8_t bytes[4], STAT, tempvar;
  bytes[0] = mag_fs & (0xFF);
  bytes[1] = (mag_fs >> 8) & (0xFF);
  bytes[2] = acc_fs & (0xFF);
  bytes[3] = (acc_fs >> 8) & (0xFF);
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte0, 1, &bytes[0], 1, 10); //Mag LSB
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte1, 1, &bytes[1], 1, 10); //Mag MSB
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte2, 1, &bytes[2], 1, 10); //Acc LSB
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte3, 1, &bytes[3], 1, 10); //Acc MSB
  tempvar = 0xCA;
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, &tempvar, 1, 10); //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
  tempvar = 0x80;
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, &tempvar, 1, 10); //Request parameter transfer procedure
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &STAT, 1, 10); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==0xCA)) {
	  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &STAT, 1, 10);
  }
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, 0x00, 1, 10); //Parameter request = 0 to end parameter transfer process
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, 0x00, 1, 10); // Re-start algorithm
}

void EM7180_set_integer_param (uint8_t param, uint32_t param_val) {
  uint8_t bytes[4], STAT, tempvar;
  bytes[0] = param_val & (0xFF);
  bytes[1] = (param_val >> 8) & (0xFF);
  bytes[2] = (param_val >> 16) & (0xFF);
  bytes[3] = (param_val >> 24) & (0xFF);
  param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte0, 1, &bytes[0], 1, 10); //Param LSB
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte1, 1, &bytes[1], 1, 10);
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte2, 1, &bytes[2], 1, 10);
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte3, 1, &bytes[3], 1, 10); //Param MSB
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, &param, 1, 10);
  tempvar = 0x80;
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, &tempvar, 1, 10); //Request parameter transfer procedure
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &STAT, 1, 10); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==param)) {
	  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &STAT, 1, 10);
  }
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, 0x00, 1, 10); //Parameter request = 0 to end parameter transfer process
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, 0x00, 1, 10); // Re-start algorithm
}

void EM7180_set_float_param (uint8_t param, float param_val) {
  uint8_t bytes[4], STAT, tempvar;
  float_to_bytes (param_val, &bytes[0]);
  param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte0, 1, &bytes[0], 1, 10); //Param LSB
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte1, 1, &bytes[1], 1, 10);
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte2, 1, &bytes[2], 1, 10);
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_LoadParamByte3, 1, &bytes[3], 1, 10); //Param MSB
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, &param, 1, 10);
  tempvar = 0x80;
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, &tempvar, 1, 10); //Request parameter transfer procedure
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &STAT, 1, 10); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==param)) {
	  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_ParamAcknowledge, 1, &STAT, 1, 10);
  }
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_ParamRequest, 1, 0x00, 1, 10); //Parameter request = 0 to end parameter transfer process
  HAL_I2C_Mem_Write(&hi2c1, EM7180_ADDRESS, EM7180_AlgorithmControl, 1, 0x00, 1, 10); // Re-start algorithm
}

void readSENtralQuatData(float * destination)
{
  uint8_t rawData[16];  // x/y/z quaternion register data stored here
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_QX, 16, &rawData[0], 16, 10);       // Read the sixteen raw data registers into data array
  destination[0] = uint32_reg_to_float (&rawData[0]);
  destination[1] = uint32_reg_to_float (&rawData[4]);
  destination[2] = uint32_reg_to_float (&rawData[8]);
  destination[3] = uint32_reg_to_float (&rawData[12]);  // SENtral stores quats as qx, qy, qz, q0!

}

void readSENtralAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_AX, 6, &rawData[0], 6, 10);       // Read the six raw data registers into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);
}

void readSENtralGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_GX, 6, &rawData[0], 6, 10);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);
}

void readSENtralMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_MX, 6, &rawData[0], 6, 10);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);
}

void getMres() {
  switch (Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0], 6, 10);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0], 6, 10);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7], tempvar;  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  tempvar = 0x01;
  if(HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_ST1, 1, &tempvar, 1, 10)) { // wait for magnetometer data ready bit to be set
  HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0], 7, 10);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
   }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0], 2, 10);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3], tempvar;  // x/y/z gyro calibration data stored here
  HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, 0x00, 1, 10); // Power down magnetometer
  osDelay(20);
  tempvar = 0x0F;
  HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, &tempvar, 1, 10); // Enter Fuse ROM access mode
  osDelay(20);
  HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0], 3, 10);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
  HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, 0x00, 1, 10); // Power down magnetometer
  osDelay(20);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  uint8_t MagDataResAndSampleODR = Mscale << 4 | Mmode;
  HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, &MagDataResAndSampleODR, 1, 10); // Set magnetometer data resolution and sample ODR
  osDelay(20);
}


void initMPU9250()
{
  uint8_t tempvar;
 // wake up device
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, 0x00, 1, 10); // Clear sleep mode bit (6), enable all sensors
  osDelay(100); // Wait for all registers to reset

 // get stable time source
  tempvar = 0x01;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, &tempvar, 1, 10);  // Auto select clock source to be PLL gyroscope reference if ready else
  osDelay(200);

 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  tempvar = 0x03;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, CONFIG, 1, &tempvar, 1, 10);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  tempvar = 0x04;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, SMPLRT_DIV, 1, &tempvar, 1, 10);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                    // determined inset in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c;
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, &c, 1, 10); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, &c, 1, 10); // Write new GYRO_CONFIG value to register

 // Set accelerometer full-scale range configuration
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &c, 1, 10); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &c, 1, 10); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &c, 1, 10); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &c, 1, 10); // Write new ACCEL_CONFIG2 register value

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   tempvar = 0x22;
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_PIN_CFG, 1, &tempvar, 1, 10);
   tempvar = 0x01;
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_ENABLE, 1, &tempvar, 1, 10);  // Enable data ready (bit 0) interrupt
   osDelay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12], tempvar; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

 // reset device
  tempvar = 0x80;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, &tempvar, 1, 10); // Write a one to bit 7 reset bit; toggle reset device
  osDelay(100);

 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
 // else use the internal oscillator, bits 2:0 = 001
  tempvar = 0x01;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, &tempvar, 1, 10);
  tempvar = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_2, 1, &tempvar, 1, 10);
  osDelay(200);

// Configure device for bias calculation
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_ENABLE, 1, &tempvar, 1, 10);   // Disable all interrupts
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, FIFO_EN, 1, &tempvar, 1, 10);      // Disable FIFO
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, &tempvar, 1, 10);   // Turn on internal clock source
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_MST_CTRL, 1, &tempvar, 1, 10); // Disable I2C master
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, &tempvar, 1, 10);    // Disable FIFO and I2C master modes
  tempvar = 0x0C;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, &tempvar, 1, 10);    // Reset FIFO and DMP
  osDelay(15);

// Configure MPU6050 gyro and accelerometer for bias calculation
  tempvar = 0x01;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, CONFIG, 1, &tempvar, 1, 10);      // Set low-pass filter to 188 Hz
  tempvar = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, SMPLRT_DIV, 1, &tempvar, 1, 10);  // Set sample rate to 1 kHz
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, &tempvar, 1, 10);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &tempvar, 1, 10); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  tempvar = 0x40;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, &tempvar, 1, 10);   // Enable FIFO
  tempvar = 0x78;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, FIFO_EN, 1, &tempvar, 1, 10);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  osDelay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  tempvar = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, FIFO_EN, 1, &tempvar, 1, 10);        // Disable gyro and accelerometer sensors for FIFO
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0], 2, 10); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, FIFO_R_W, 12, &data[0], 12, 10); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, XG_OFFSET_H, 1, &data[0], 1, 10);
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, XG_OFFSET_L, 1, &data[1], 1, 10);
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, YG_OFFSET_H, 1, &data[2], 1, 10);
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, YG_OFFSET_L, 1, &data[3], 1, 10);
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ZG_OFFSET_H, 1, &data[4], 1, 10);
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ZG_OFFSET_L, 1, &data[5], 1, 10);

// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0], 2, 10); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0], 2, 10);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0], 2, 10);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFE;
  data[1] = (accel_bias_reg[0])      & 0xFE;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFE;
  data[3] = (accel_bias_reg[1])      & 0xFE;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFE;
  data[5] = (accel_bias_reg[2])      & 0xFE;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


void magcalMPU9250(float * dest1, float * dest2)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {0xFF, 0xFF, 0xFF}, mag_min[3] = {0x7F, 0x7F, 0x7F}, mag_temp[3] = {0, 0, 0};

  sprintf(buffer, "Mag Cal: Wave device in figure 8");
  HAL_UART_Transmit(&huart2, buffer, 58, 10);
  osDelay(4000);

   if(Mmode == 0x02) sample_count = 128;
   if(Mmode == 0x06) sample_count = 1500;
   for(ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if(Mmode == 0x02) osDelay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) osDelay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
   }

//    sprintf(buffer, "mag x min/max:"); sprintf(buffer, mag_max[0]); sprintf(buffer, mag_min[0]);
//    sprintf(buffer, "mag y min/max:"); sprintf(buffer, mag_max[1]); sprintf(buffer, mag_min[1]);
//    sprintf(buffer, "mag z min/max:"); sprintf(buffer, mag_max[2]); sprintf(buffer, mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
    dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

   sprintf(buffer, "Mag Calibration done!");
   HAL_UART_Transmit(&huart2, buffer, 21, 10);
}




// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
   float factoryTrim[6];
   uint8_t FS = 0;
   uint8_t tempvar;
  tempvar = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, SMPLRT_DIV, 1, &tempvar, 1, 10);    // Set gyro sample rate to 1 kHz
  tempvar = 0x02;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, CONFIG, 1, &tempvar, 1, 10);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  uint8_t FSshifted = FS<<3;
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, &FSshifted, 1, 10);  // Set full scale range for the gyro to 250 dps
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &tempvar, 1, 10); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &FSshifted, 1, 10); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0], 6, 10);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0], 6, 10);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }

// Configure the accelerometer for self-test
  tempvar = 0xE0;
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &tempvar, 1, 10); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, &tempvar, 1, 10); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   osDelay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and accelerometer

  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0], 6, 10);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0], 6, 10);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }

 // Configure the gyro and accelerometer for normal operation
  tempvar = 0x00;
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &tempvar, 1, 10);
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, &tempvar, 1, 10);
   osDelay(25);  // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_X_ACCEL, 1, &selfTest[0], 1, 10); // X-axis accel self-test results
   HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_Y_ACCEL, 1, &selfTest[1], 1, 10); // Y-axis accel self-test results
   HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_Z_ACCEL, 1, &selfTest[2], 1, 10); // Z-axis accel self-test results
   HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_X_GYRO, 1, &selfTest[3], 1, 10);  // X-axis gyro self-test results
   HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_Y_GYRO, 1, &selfTest[4], 1, 10);  // Y-axis gyro self-test results
   HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, SELF_TEST_Z_GYRO, 1, &selfTest[5], 1, 10);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
   }

}

int16_t readSENtralBaroData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_Baro, 2, &rawData[0], 2, 10);  // Read the two raw data registers sequentially into data array
  return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}

int16_t readSENtralTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  HAL_I2C_Mem_Read(&hi2c1, EM7180_ADDRESS, EM7180_Temp, 2, &rawData[0], 2, 10);  // Read the two raw data registers sequentially into data array
  return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
}

int32_t readBMP280Temperature()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDRESS, BMP280_TEMP_MSB, 3, &rawData[0], 3, 10);
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

int32_t readBMP280Pressure()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDRESS, BMP280_PRESS_MSB, 3, &rawData[0], 3, 10);
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

void BMP280Init()
{
  // Configure the BMP280
  // Set T and P oversampling rates and sensor mode
	uint8_t TPosrMode = Tosr << 5 | Posr << 2 | Mode;
  HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDRESS, BMP280_CTRL_MEAS, 1, &TPosrMode, 1, 10);
  // Set standby time interval in normal mode and bandwidth
  uint8_t SByTimeIntervalModeAndBandwidth = SBy << 5 | IIRFilter << 2;
  HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDRESS, BMP280_CONFIG, 1, &SByTimeIntervalModeAndBandwidth, 1, 10);
  // Read and store calibration data
  uint8_t calib[24];
  HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDRESS, BMP280_CALIB00, 24, &calib[0], 24, 10);
  dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
  dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
  dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
  dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
  dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
  dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
  dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
  dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
  dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
  dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
  dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
  dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// �5123� equals 51.23 DegC.
int32_t bmp280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
//fractional bits).
//Output value of �24674867� represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P(int32_t adc_P)
{
  long long var1, var2, p;
  var1 = ((long long)t_fine) - 128000;
  var2 = var1 * var1 * (long long)dig_P6;
  var2 = var2 + ((var1*(long long)dig_P5)<<17);
  var2 = var2 + (((long long)dig_P4)<<35);
  var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
  var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
  if(var1 == 0)
  {
    return 0;
    // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125)/var1;
  var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((long long)dig_P8) * p)>> 19;
  p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
  return (uint32_t)p;
}

/* Copyright 2017 Bolder Flight Systems <brian.taylor@bolderflight.com>.
 * Copyright 2018, Sergio Renato De Jesus Melean <sergiordj@gmail.com>.
 * Copyright 2018, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
/* Date: 2018-07-06 */

/*==================[inclusions]=============================================*/

#include "mpu6500.h"   /* <= sAPI HMC5883L header */
#include "i2c.h"
//#include "sapi_i2c.h"           /* <= sAPI I2C header */
//#include "sapi_delay.h"         /* <= sAPI Delay header */

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static void mpu6500InitializeControlStructure( void );
static int8_t mpu6500WriteRegister( uint8_t subAddress, uint8_t data );
static int8_t mpu6500ReadRegisters( uint8_t subAddress, uint8_t count );
static int8_t mpu6500WhoAmI( void );
static int8_t mpu6500CalibrateGyro( void );
static int8_t mpu6500SetGyroRange( MPU6500_GyroRange_t range );
static int8_t mpu6500SetDlpfBandwidth( MPU6500_DlpfBandwidth_t bandwidth );
static int8_t mpu6500SetSrd( uint8_t srd );

/*==================[internal data definition]===============================*/

//MPU control structure
static MPU6500_control_t control;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void mpu6500InitializeControlStructure( void )
{
	control._tempScale = 333.87f;
	control._tempOffset = 21.0f;
	control._numSamples = 100;
	control._axs = 1.0f;
	control._ays = 1.0f;
	control._azs = 1.0f;
	control.tX[0] = 0;
	control.tX[1] = 1;
	control.tX[2] = 0;
	control.tY[0] = 1;
	control.tY[1] = 0;
	control.tY[2] = 0;
	control.tZ[0] = 0;
	control.tZ[1] = 0;
	control.tZ[2] = -1;
}

static int8_t mpu6500WriteRegister( uint8_t command, uint8_t data ) //subAddress = command.
{
	uint8_t buff[2];
	buff[0] = command;
	buff[1] = data;
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(&hi2c1, control.address<<1, buff, 2, HAL_MAX_DELAY);
	if (status != HAL_OK)
		return -1;
	return 1;

	//HAL_Delay(10);
}

static int8_t mpu6500ReadRegisters( uint8_t command, uint8_t count )
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(&hi2c1, control.address<<1, &command, 1, HAL_MAX_DELAY);
	if (status != HAL_OK)
		return -1;
	status = HAL_I2C_Master_Receive(&hi2c1, control.address<<1, control._buffer, count, HAL_MAX_DELAY);
	if (status != HAL_OK)
		return -1;
	return 1;
}

static int8_t mpu6500WhoAmI( void )
{
	// read the WHO AM I register
	if (mpu6500ReadRegisters(MPU6500_WHO_AM_I,1) < 0) {
		return -1;
	}
	// return the register value
	return control._buffer[0];
}

static int8_t mpu6500CalibrateGyro( void )
{
	// set the range, bandwidth, and srd
	if (mpu6500SetGyroRange(MPU6500_GYRO_RANGE_250DPS) < 0) {
		return -1;
	}
	if (mpu6500SetDlpfBandwidth(MPU6500_DLPF_BANDWIDTH_20HZ) < 0) {
		return -2;
	}
	if (mpu6500SetSrd(19) < 0) {
		return -3;
	}

	// take samples and find bias
	control._gxbD = 0;
	control._gybD = 0;
	control._gzbD = 0;
	for (uint8_t i=0; i < control._numSamples; i++) {
		mpu6500Read();
		control._gxbD += ((mpu6500GetGyroX_rads() + control._gxb)/control._numSamples);
		control._gybD += ((mpu6500GetGyroY_rads() + control._gyb)/control._numSamples);
		control._gzbD += ((mpu6500GetGyroZ_rads() + control._gzb)/control._numSamples);
		HAL_Delay(20);
	}
	control._gxb = (float)control._gxbD;
	control._gyb = (float)control._gybD;
	control._gzb = (float)control._gzbD;

	// set the range, bandwidth, and srd back to what they were
	if (mpu6500SetGyroRange(control._gyroRange) < 0) {
		return -4;
	}
	if (mpu6500SetDlpfBandwidth(control._bandwidth) < 0) {
		return -5;
	}
	if (mpu6500SetSrd(control._srd) < 0) {
		return -6;
	}
	return 1;
}

static int8_t mpu6500SetGyroRange( MPU6500_GyroRange_t range )
{
	switch(range) {
		case MPU6500_GYRO_RANGE_250DPS: {
		  // setting the gyro range to 250DPS
		  if(mpu6500WriteRegister(MPU6500_GYRO_CONFIG, MPU6500_GYRO_FS_SEL_250DPS) < 0){
			return -1;
		  }
        // setting the gyro scale to 250DPS
		  control._gyroScale = 250.0f/32767.5f * MPU6500_D2R;
		  break;
		}
		case MPU6500_GYRO_RANGE_500DPS: {
		  // setting the gyro range to 500DPS
		  if(mpu6500WriteRegister(MPU6500_GYRO_CONFIG, MPU6500_GYRO_FS_SEL_500DPS) < 0){
			return -1;
		  }
        // setting the gyro scale to 500DPS
		  control._gyroScale = 500.0f/32767.5f * MPU6500_D2R;
		  break;
		}
		case MPU6500_GYRO_RANGE_1000DPS: {
		  // setting the gyro range to 1000DPS
		  if(mpu6500WriteRegister(MPU6500_GYRO_CONFIG, MPU6500_GYRO_FS_SEL_1000DPS) < 0){
			return -1;
		  }
        // setting the gyro scale to 1000DPS
		  control._gyroScale = 1000.0f/32767.5f * MPU6500_D2R;
		  break;
		}
		case MPU6500_GYRO_RANGE_2000DPS: {
		  // setting the gyro range to 2000DPS
		  if(mpu6500WriteRegister(MPU6500_GYRO_CONFIG, MPU6500_GYRO_FS_SEL_2000DPS) < 0){
			return -1;
		  }
        // setting the gyro scale to 2000DPS
		  control._gyroScale = 2000.0f/32767.5f * MPU6500_D2R;
		  break;
		}
	}
	control._gyroRange = range;
	return 1;
}

static int8_t mpu6500SetDlpfBandwidth( MPU6500_DlpfBandwidth_t bandwidth )
{
	switch (bandwidth) {
		case MPU6500_DLPF_BANDWIDTH_184HZ: {
         // setting accel bandwidth to 184Hz
			if (mpu6500WriteRegister(MPU6500_ACCEL_CONFIG2, MPU6500_ACCEL_DLPF_184) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 184Hz
			if (mpu6500WriteRegister(MPU6500_CONFIG, MPU6500_GYRO_DLPF_184) < 0) {
				return -2;
			}
			break;
		}
		case MPU6500_DLPF_BANDWIDTH_92HZ: {
         // setting accel bandwidth to 92Hz
			if (mpu6500WriteRegister(MPU6500_ACCEL_CONFIG2, MPU6500_ACCEL_DLPF_92) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 92Hz
			if (mpu6500WriteRegister(MPU6500_CONFIG, MPU6500_GYRO_DLPF_92) < 0) {
				return -2;
			}
			break;
		}
		case MPU6500_DLPF_BANDWIDTH_41HZ: {
         // setting accel bandwidth to 41Hz
			if (mpu6500WriteRegister(MPU6500_ACCEL_CONFIG2, MPU6500_ACCEL_DLPF_41) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 41Hz
			if (mpu6500WriteRegister(MPU6500_CONFIG, MPU6500_GYRO_DLPF_41) < 0) {
				return -2;
			}
			break;
		}
		case MPU6500_DLPF_BANDWIDTH_20HZ: {
         // setting accel bandwidth to 20Hz
			if (mpu6500WriteRegister(MPU6500_ACCEL_CONFIG2, MPU6500_ACCEL_DLPF_20) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 20Hz
			if (mpu6500WriteRegister(MPU6500_CONFIG, MPU6500_GYRO_DLPF_20) < 0) {
				return -2;
			}
			break;
		}
		case MPU6500_DLPF_BANDWIDTH_10HZ: {
         // setting accel bandwidth to 10Hz
			if (mpu6500WriteRegister(MPU6500_ACCEL_CONFIG2, MPU6500_ACCEL_DLPF_10) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 10Hz
			if (mpu6500WriteRegister(MPU6500_CONFIG, MPU6500_GYRO_DLPF_10) < 0) {
				return -2;
			}
			break;
		}
		case MPU6500_DLPF_BANDWIDTH_5HZ: {
         // setting accel bandwidth to 5Hz
			if (mpu6500WriteRegister(MPU6500_ACCEL_CONFIG2, MPU6500_ACCEL_DLPF_5) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 5Hz
			if (mpu6500WriteRegister(MPU6500_CONFIG, MPU6500_GYRO_DLPF_5) < 0) {
				return -2;
			}
			break;
		}
	}
	control._bandwidth = bandwidth;
	return 1;
}

static int8_t mpu6500SetSrd( uint8_t srd )
{
	/* setting the sample rate divider to 19 to facilitate setting up 
      magnetometer */
   // setting the sample rate divider
	if (mpu6500WriteRegister(MPU6500_SMPDIV, 19) < 0) {
		return -1;
	}
	/* setting the sample rate divider */
	if (mpu6500WriteRegister(MPU6500_SMPDIV, srd) < 0) { // setting the sample rate divider
		return -4;
	}
	control._srd = srd;
	return 1;
}

/*==================[external functions definition]==========================*/

//Initialize MPU6500 (TODO: include SPI communication)
int8_t mpu6500Init( MPU6500_address_t address )
{
	mpu6500InitializeControlStructure();

	control.address = address;

	// using I2C for communication
	// starting the I2C bus
	MX_I2C1_Init(); //i2cInit(I2C0, MPU6500_I2C_RATE);

	// select clock source to gyro
	if (mpu6500WriteRegister(MPU6500_PWR_MGMNT_1, MPU6500_CLOCK_SEL_PLL) < 0) {
		return -1;
	}
	// enable I2C master mode
	if (mpu6500WriteRegister(MPU6500_USER_CTRL, MPU6500_I2C_MST_EN) < 0) {
		return -2;
	}
	// set the I2C bus speed to 400 kHz
	if (mpu6500WriteRegister(MPU6500_I2C_MST_CTRL, MPU6500_I2C_MST_CLK) < 0) {
		return -3;
	}
	// reset the MPU6500
	mpu6500WriteRegister(MPU6500_PWR_MGMNT_1, MPU6500_PWR_RESET);
	// wait for MPU-6500 to come back up
	HAL_Delay(1);
	// select clock source to gyro
	if (mpu6500WriteRegister(MPU6500_PWR_MGMNT_1, MPU6500_CLOCK_SEL_PLL) < 0) {
		return -4;
	}
	// check the WHO AM I byte, expected value is 0x70 (decimal 112) or 0x73 (decimal 115)
	if ((mpu6500WhoAmI() != 112) && (mpu6500WhoAmI() != 115)) {
		return -5;
	}
	// enable accelerometer and gyro
	if (mpu6500WriteRegister(MPU6500_PWR_MGMNT_2, MPU6500_SEN_ENABLE) < 0) {
		return -6;
	}
	// setting accel range to 16G as default
	if (mpu6500WriteRegister(MPU6500_ACCEL_CONFIG, MPU6500_ACCEL_FS_SEL_16G) < 0) {
		return -7;
	}
	control._accelScale = MPU6500_G * 16.0f / 32767.5f; // setting the accel scale to 16G
	control._accelRange = MPU6500_ACCEL_RANGE_16G;
	// setting the gyro range to 2000DPS as default
	if (mpu6500WriteRegister(MPU6500_GYRO_CONFIG, MPU6500_GYRO_FS_SEL_2000DPS) < 0) {
		return -8;
	}
   // setting the gyro scale to 2000DPS
	control._gyroScale = 2000.0f / 32767.5f * MPU6500_D2R;
	control._gyroRange = MPU6500_GYRO_RANGE_2000DPS;
	// setting bandwidth to 184Hz as default
	if (mpu6500WriteRegister(MPU6500_ACCEL_CONFIG2, MPU6500_ACCEL_DLPF_184) < 0) {
		return -9;
	}
   // setting gyro bandwidth to 184Hz
	if (mpu6500WriteRegister(MPU6500_CONFIG, MPU6500_GYRO_DLPF_184) < 0) {
		return -10;
	}
	control._bandwidth = MPU6500_DLPF_BANDWIDTH_184HZ;
	// setting the sample rate divider to 0 as default
	if (mpu6500WriteRegister(MPU6500_SMPDIV, 0x00) < 0) {
		return -11;
	}
	control._srd = 0;
	// enable I2C master mode
	if (mpu6500WriteRegister(MPU6500_USER_CTRL, MPU6500_I2C_MST_EN) < 0) {
		return -12;
	}
	// set the I2C bus speed to 400 kHz
	if (mpu6500WriteRegister(MPU6500_I2C_MST_CTRL, MPU6500_I2C_MST_CLK) < 0) {
		return -13;
	}
	HAL_Delay(100); // long wait between AK8963 mode changes
	// select clock source to gyro
	if (mpu6500WriteRegister(MPU6500_PWR_MGMNT_1, MPU6500_CLOCK_SEL_PLL) < 0) {
		return -19;
	}
	// estimate gyro bias
	if (mpu6500CalibrateGyro() < 0) {
		return -20;
	}
	// successful init, return 1
	return 1;
}

//Read sensor registers and store data at control structure
bool mpu6500Read(void)
{
	// grab the data from the MPU6500
	if( !mpu6500ReadRegisters(MPU6500_ACCEL_OUT, 21) ){
		return 0;
	}
	// combine into 16 bit values
	control._axcounts = (((int16_t)control._buffer[0]) << 8)  | control._buffer[1];
	control._aycounts = (((int16_t)control._buffer[2]) << 8)  | control._buffer[3];
	control._azcounts = (((int16_t)control._buffer[4]) << 8)  | control._buffer[5];
	control._tcounts  = (((int16_t)control._buffer[6]) << 8)  | control._buffer[7];
	control._gxcounts = (((int16_t)control._buffer[8]) << 8)  | control._buffer[9];
	control._gycounts = (((int16_t)control._buffer[10]) << 8) | control._buffer[11];
	control._gzcounts = (((int16_t)control._buffer[12]) << 8) | control._buffer[13];
	// transform and convert to float values
	control._ax = (((float)(control.tX[0]*control._axcounts + control.tX[1]*control._aycounts + control.tX[2]*control._azcounts) * control._accelScale) - control._axb)*control._axs;
	control._ay = (((float)(control.tY[0]*control._axcounts + control.tY[1]*control._aycounts + control.tY[2]*control._azcounts) * control._accelScale) - control._ayb)*control._ays;
	control._az = (((float)(control.tZ[0]*control._axcounts + control.tZ[1]*control._aycounts + control.tZ[2]*control._azcounts) * control._accelScale) - control._azb)*control._azs;
	control._gx = ((float) (control.tX[0]*control._gxcounts + control.tX[1]*control._gycounts + control.tX[2]*control._gzcounts) * control._gyroScale) -  control._gxb;
	control._gy = ((float) (control.tY[0]*control._gxcounts + control.tY[1]*control._gycounts + control.tY[2]*control._gzcounts) * control._gyroScale) -  control._gyb;
	control._gz = ((float) (control.tZ[0]*control._gxcounts + control.tZ[1]*control._gycounts + control.tZ[2]*control._gzcounts) * control._gyroScale) -  control._gzb;
	control._t = ((((float) control._tcounts)  - control._tempOffset)/ control._tempScale) + control._tempOffset;
	return 1;
}

// Returns the accelerometer measurement in the x direction, m/s/s
float mpu6500GetAccelX_mss( void )
{
	return control._ax;
}

// Returns the accelerometer measurement in the y direction, m/s/s
float mpu6500GetAccelY_mss( void )
{
	return control._ay;
}

// Returns the accelerometer measurement in the z direction, m/s/s
float mpu6500GetAccelZ_mss( void )
{
	return control._az;
}

// Returns the gyroscope measurement in the x direction, rad/s
float mpu6500GetGyroX_rads( void )
{
	return control._gx;
}

// Returns the gyroscope measurement in the y direction, rad/s
float mpu6500GetGyroY_rads( void )
{
	return control._gy;
}

// Returns the gyroscope measurement in the z direction, rad/s
float mpu6500GetGyroZ_rads( void )
{
	return control._gz;
}

// Returns the die temperature, C
float mpu6500GetTemperature_C( void )
{
  return control._t;
}

/*==================[end of file]============================================*/

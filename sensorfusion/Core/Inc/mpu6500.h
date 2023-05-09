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

#ifndef _SAPI_IMU_MPU6500_H_
#define _SAPI_IMU_MPU6500_H_

/*==================[inclusions]=============================================*/

//#include "sapi_datatypes.h"
#include "stdint.h"
#include "stdbool.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
// physical constants
#define MPU6500_G                     9.807f
#define MPU6500_D2R                   3.14159265359f/180.0f

// MPU6500 registers
#define MPU6500_ACCEL_OUT             0x3B
#define MPU6500_GYRO_OUT              0x43
#define MPU6500_TEMP_OUT              0x41
#define MPU6500_EXT_SENS_DATA_00      0x49
#define MPU6500_ACCEL_CONFIG 	      0x1C
#define MPU6500_ACCEL_FS_SEL_2G       0x00
#define MPU6500_ACCEL_FS_SEL_4G       0x08
#define MPU6500_ACCEL_FS_SEL_8G       0x10
#define MPU6500_ACCEL_FS_SEL_16G      0x18
#define MPU6500_GYRO_CONFIG           0x1B
#define MPU6500_GYRO_FS_SEL_250DPS    0x00
#define MPU6500_GYRO_FS_SEL_500DPS    0x08
#define MPU6500_GYRO_FS_SEL_1000DPS   0x10
#define MPU6500_GYRO_FS_SEL_2000DPS   0x18

#define MPU6500_ACCEL_CONFIG2         0x1D
#define MPU6500_ACCEL_DLPF_184        0x01
#define MPU6500_ACCEL_DLPF_92         0x02
#define MPU6500_ACCEL_DLPF_41         0x03
#define MPU6500_ACCEL_DLPF_20         0x04
#define MPU6500_ACCEL_DLPF_10         0x05
#define MPU6500_ACCEL_DLPF_5          0x06
#define MPU6500_CONFIG                0x1A
#define MPU6500_GYRO_DLPF_184         0x01
#define MPU6500_GYRO_DLPF_92          0x02
#define MPU6500_GYRO_DLPF_41          0x03
#define MPU6500_GYRO_DLPF_20          0x04
#define MPU6500_GYRO_DLPF_10          0x05
#define MPU6500_GYRO_DLPF_5           0x06
#define MPU6500_SMPDIV                0x19
#define MPU6500_INT_PIN_CFG           0x37
#define MPU6500_INT_ENABLE            0x38
#define MPU6500_INT_DISABLE           0x00
#define MPU6500_INT_PULSE_50US        0x00
#define MPU6500_INT_WOM_EN            0x40
#define MPU6500_INT_RAW_RDY_EN        0x01
#define MPU6500_PWR_MGMNT_1           0x6B
#define MPU6500_PWR_CYCLE             0x20
#define MPU6500_PWR_RESET             0x80
#define MPU6500_CLOCK_SEL_PLL         0x01
#define MPU6500_PWR_MGMNT_2           0x6C
#define MPU6500_SEN_ENABLE            0x00
#define MPU6500_DIS_GYRO              0x07
#define MPU6500_USER_CTRL             0x6A
#define MPU6500_I2C_MST_EN            0x20
#define MPU6500_I2C_MST_CLK           0x0D
#define MPU6500_I2C_MST_CTRL          0x24
#define MPU6500_I2C_SLV0_ADDR         0x25
#define MPU6500_I2C_SLV0_REG          0x26
#define MPU6500_I2C_SLV0_DO           0x63
#define MPU6500_I2C_SLV0_CTRL         0x27
#define MPU6500_I2C_SLV0_EN           0x80
#define MPU6500_I2C_READ_FLAG         0x80
#define MPU6500_MOT_DETECT_CTRL       0x69
#define MPU6500_ACCEL_INTEL_EN        0x80
#define MPU6500_ACCEL_INTEL_MODE      0x40
#define MPU6500_LP_ACCEL_ODR          0x1E
#define MPU6500_WOM_THR               0x1F
#define MPU6500_WHO_AM_I              0x75
#define MPU6500_FIFO_EN               0x23
#define MPU6500_FIFO_TEMP             0x80
#define MPU6500_FIFO_GYRO             0x70
#define MPU6500_FIFO_ACCEL            0x08
#define MPU6500_FIFO_MAG              0x01
#define MPU6500_FIFO_COUNT            0x72
#define MPU6500_FIFO_READ             0x74

/*==================[typedef]================================================*/

//Different options for basic MPU6500 setting registers

typedef enum {
   MPU6500_ADDRESS_0 = 0x68,
   MPU6500_ADDRESS_1 = 0x69
} MPU6500_address_t;

typedef enum
{
   MPU6500_ACCEL_RANGE_2G,
   MPU6500_ACCEL_RANGE_4G,
   MPU6500_ACCEL_RANGE_8G,
   MPU6500_ACCEL_RANGE_16G
} MPU6500_AccelRange_t;

typedef enum
{
   MPU6500_GYRO_RANGE_250DPS,
   MPU6500_GYRO_RANGE_500DPS,
   MPU6500_GYRO_RANGE_1000DPS,
   MPU6500_GYRO_RANGE_2000DPS
} MPU6500_GyroRange_t;

typedef enum
{
   MPU6500_DLPF_BANDWIDTH_184HZ,
   MPU6500_DLPF_BANDWIDTH_92HZ,
   MPU6500_DLPF_BANDWIDTH_41HZ,
   MPU6500_DLPF_BANDWIDTH_20HZ,
   MPU6500_DLPF_BANDWIDTH_10HZ,
   MPU6500_DLPF_BANDWIDTH_5HZ
} MPU6500_DlpfBandwidth_t;

typedef enum
{
   MPU6500_LP_ACCEL_ODR_0_24HZ  = 0,
   MPU6500_LP_ACCEL_ODR_0_49HZ  = 1,
   MPU6500_LP_ACCEL_ODR_0_98HZ  = 2,
   MPU6500_LP_ACCEL_ODR_1_95HZ  = 3,
   MPU6500_LP_ACCEL_ODR_3_91HZ  = 4,
   MPU6500_LP_ACCEL_ODR_7_81HZ  = 5,
   MPU6500_LP_ACCEL_ODR_15_63HZ = 6,
   MPU6500_LP_ACCEL_ODR_31_25HZ = 7,
   MPU6500_LP_ACCEL_ODR_62_50HZ = 8,
   MPU6500_LP_ACCEL_ODR_125HZ   = 9,
   MPU6500_LP_ACCEL_ODR_250HZ   = 10,
   MPU6500_LP_ACCEL_ODR_500HZ   = 11
} MPU6500_LpAccelOdr_t;

//Control structure for MPU6500 operation (only one IMU per project)
typedef struct {
   MPU6500_address_t address; //MPU6500 address can be configured through AD0 pin
   
   // scale factors
   float _accelScale;
   float _gyroScale;
   float _magScaleX;
   float _magScaleY;
   float _magScaleZ;
   float _tempScale;
   float _tempOffset;
   
   // configuration
   MPU6500_AccelRange_t    _accelRange;
   MPU6500_GyroRange_t     _gyroRange;
   MPU6500_DlpfBandwidth_t _bandwidth;
   uint8_t _srd;

   // buffer for reading from sensor
   uint8_t _buffer[14];

   // data buffer
   float _ax, _ay, _az;
   float _gx, _gy, _gz;
   float _t;

   // gyro bias estimation
   uint8_t _numSamples;
   double _gxbD, _gybD, _gzbD;
   float _gxb, _gyb, _gzb;

   // accel bias and scale factor estimation
   double _axbD, _aybD, _azbD;
   float _axmax, _aymax, _azmax;
   float _axmin, _aymin, _azmin;
   float _axb, _ayb, _azb;
   float _axs;
   float _ays;
   float _azs;

   // data counts
   int16_t _axcounts, _aycounts, _azcounts;
   int16_t _gxcounts, _gycounts, _gzcounts;
   int16_t _tcounts;

   // transformation matrix
   /* transform the accel and gyro axes to match the magnetometer axes */
   int16_t tX[3];
   int16_t tY[3];
   int16_t tZ[3];

   // track success of interacting with sensor
   int8_t _status;

} MPU6500_control_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

// Initialize MPU6500 (Only I2C)
int8_t mpu6500Init( MPU6500_address_t address );

// Read sensor registers and store data at control structure
bool mpu6500Read(void);


// Returns the accelerometer measurement in the x direction, m/s/s
float mpu6500GetAccelX_mss( void );

// Returns the accelerometer measurement in the y direction, m/s/s
float mpu6500GetAccelY_mss( void );

// Returns the accelerometer measurement in the z direction, m/s/s
float mpu6500GetAccelZ_mss( void );

// Returns the gyroscope measurement in the x direction, rad/s
float mpu6500GetGyroX_rads( void );

// Returns the gyroscope measurement in the y direction, rad/s
float mpu6500GetGyroY_rads( void );

// Returns the gyroscope measurement in the z direction, rad/s
float mpu6500GetGyroZ_rads( void );

// Returns the die temperature, C
float mpu6500GetTemperature_C( void );

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_IMU_MPU6500_H_ */

/*
 * peripherals.h
 *
 *  Created on: Feb 23, 2014
 *      Author: michael
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#define ABS(x)         (x < 0) ? (-x) : x
#define PI                      (float)     3.14159265f     //PI IS EXACTLY THREE!!!
#define g						(float) 	9.80665f		//Norm. earth acceleration [m/s^2]

// Regarding value scaling, the SDK expects the sensor values to be sent as 21 bit signed integer
// Furthermore, it expects these values to have a physical unit ...
// This can be found in OVR_SensorImpl in the SDK:
//--------------------------------------------------------------------------
// Sensor reports data in the following coordinate system:
// Accelerometer: 10^-4 m/s^2; X forward, Y right, Z Down.
// Gyro:          10^-4 rad/s; X positive roll right, Y positive pitch up; Z positive yaw right.
// Magnetic field strength in 10^-4 Gauss.
//--------------------------------------------------------------------------

//Accelerometer Conversion factors to get [10^-4 m/s^2]
//--------------------------------------------------------------------------
//First value is sensor sensitivity in [mg/digit] as taken from datasheet
//Last /16 is needed, as 12 bit values are left aligned in the 16 bit fields
#define ACC_SCALE_FACTOR_2 		( 1.0f * g * 10 )/16
#define ACC_SCALE_FACTOR_4   	( 2.0f * g * 10 )/16
#define ACC_SCALE_FACTOR_8   	( 4.0f * g * 10 )/16
#define ACC_SCALE_FACTOR_16     (12.0f * g * 10 )/16

//Gyro Conversion factors to get [10^-4 rad/s]
//--------------------------------------------------------------------------
//First value is sensitivity in [mdeg/digit] as taken from data sheet
#define GYRO_SCALE_FACTOR_250	(  8.75f / 360 * 2 * PI * 10 )
#define GYRO_SCALE_FACTOR_500	( 17.50f / 360 * 2 * PI * 10 )
#define GYRO_SCALE_FACTOR_2000  ( 70.00f / 360 * 2 * PI * 10 )

//Magneto Conversion factors to get [10^-4 G]
//--------------------------------------------------------------------------
//First value is sensitivity in [m Gauss/digit] as taken from data sheet
//Here those ST monkeys have put a different gain value for X,Y  and the Z axis.
#define MAG_SCALE_FACTOR_1300_XY ( 1E4 / 1100 )
#define MAG_SCALE_FACTOR_1300_Z  ( 1E4 / 980 )
#define MAG_SCALE_FACTOR_1900_XY ( 1E4 / 855 )
#define MAG_SCALE_FACTOR_1900_Z  ( 1E4 / 760 )
#define MAG_SCALE_FACTOR_2500_XY ( 1E4 / 670 )
#define MAG_SCALE_FACTOR_2500_Z  ( 1E4 / 600 )

//Calibration and permanent storage in FLASH
//--------------------------------------------------------------------------
#define FLASH_USER_START_ADDR   0x08006000   // Start @ of user Flash area (Program code goes to 0x08005268)
void restoreCalibrationFromFlash();
void saveCalibrationToFlash();


void initSensorGyro( uint16_t gyroScale );
void doGyroBiasCalibration();
uint8_t isGyroNewData();
int16_t readSensorGyro( int32_t* pfData );
void initSensorAccMag( uint8_t accelerometerScale, uint16_t magnetomaterScale );
uint8_t isAccNewData();
void readSensorAcc( int32_t* pfData );
uint8_t isMagNewData();
int16_t readSensorMag( int16_t* pfData );

void delayms( uint32_t delay );

void LED_Config(void);
void LED_out_byte( uint8_t dat );
void LED_out( uint8_t dat );
void LEDs_Off(void);
void LED_ani_idle();
void LED_ani_run();

#endif /* PERIPHERALS_H_ */

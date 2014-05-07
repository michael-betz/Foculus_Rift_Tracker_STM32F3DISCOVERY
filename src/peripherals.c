/*
 * peripherals.c
 *
 *  Created on: Feb 23, 2014
 *      Author: michael
 */
#include "main.h"
#include "peripherals.h"
#include "globalVariables.h"
#include "stm32f30x_it.h"
#include <string.h>

//These variables are counted down to 0 by the timer interrupt (every 1 ms)
__IO uint32_t timerTimeSinceLastKeepAlive = 0;
__IO uint32_t timerTimeSinceLastFrameSent = 0;
__IO uint32_t timerTimeSinceLastAccMagReadout = 0;
__IO uint32_t timerDelayCountdown = 0;

uint16_t	currentGyroScale;	//Either 250, 500 or 2000, set by initGyro(), used by calibration routine

//The scaling factors are used to convert arbitary digits from the 3 sensors to physical units.
//In particular to [10^-4 rad/s], [10^-4 m/s^2] and [10^-4 Gauss]
//As the factors depend on the current sensor scale, the values are set by the initSensor functions.
float gyroScalingFactors[3];	//X,Y,Z
float accScalingFactors[3];		//X,Y,Z
float magScalingFactors[3];		//X,Z,Y

//To correct the gyro offset error, these values are subtracted from the result
float gyroBiasCalValue250[3] = { 0.0, 0.0, 0.0 };
float gyroBiasCalValue500[3] = { 0.0, 0.0, 0.0 };
float gyroBiasCalValue2000[3] = { 0.0, 0.0, 0.0 };
float *gyroBiasCalValue;		//Will always point to the relevant one of the three arrays ( set by initGyro() )

//-------------------------------------------------------------------------------------------
// Timer interrupt, called every 1 ms, keeps track of some counting variables
//-------------------------------------------------------------------------------------------
void SysTick_Handler(void) {
	timerTimeSinceLastKeepAlive++;
	timerTimeSinceLastFrameSent++;
	timerTimeSinceLastAccMagReadout++;
	if (timerDelayCountdown > 0){
		timerDelayCountdown--;
	}
}

//do nothing for _delay_ ms
void delayms( uint32_t delay ){
	timerDelayCountdown = delay;
	while( timerDelayCountdown );
}

// Little convenience function to set all 3 scale factors (X,Y,Z) to the same value
void _setAll3ScalingFactors( float *scaleFactorsToSet, float scaleValue ){
	uint8_t i;
	for(i=0; i<=2; i++){
		scaleFactorsToSet[i] = scaleValue;
	}
}

//-------------------------------------------------------------------------------------------
// * @brief  Configure the L3GD20 Mems gyroscope sensor and set scaling values
// * @param  gyroScale set by this in the SDK:  GyroRangeRamp[]  = { 250, 500, 1000, 2000 };
//-------------------------------------------------------------------------------------------
void initSensorGyro(uint16_t gyroScale) {
	uint8_t tempValue;
	L3GD20_InitTypeDef L3GD20_InitStructure;
	currentGyroScale = gyroScale;
	/* Configure Mems L3GD20 */
	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_4;//760 Hz
	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;			//Max bandwidth
	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Single;
	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	switch (gyroScale) {
	case 250:
		L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_250;
		_setAll3ScalingFactors( gyroScalingFactors, GYRO_SCALE_FACTOR_250 );
		gyroBiasCalValue = gyroBiasCalValue250;
		break;
	case 500:
	case 1000:	//1000 deg/s fullscale is not supported by ST chip, use 500 deg/s instead
		L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500;
		_setAll3ScalingFactors( gyroScalingFactors, GYRO_SCALE_FACTOR_500 );
		gyroBiasCalValue = gyroBiasCalValue500;
		break;
	case 2000:
	default:
		L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_2000;
		_setAll3ScalingFactors( gyroScalingFactors, GYRO_SCALE_FACTOR_2000 );
		gyroBiasCalValue = gyroBiasCalValue2000;
	}
	L3GD20_Init(&L3GD20_InitStructure);	//Writes to CTRL_REG1 and CTRL_REG4

	tempValue = 0b00000001;	//No INT1, No boot status on INT1 pin, INT1 inactive, push-pull, No DRDY on INT2 pin, No INT2 FIFO watermakr, No INT2 FIFO overrun, INT2 FIFO empty = on
	L3GD20_Write(&tempValue, L3GD20_CTRL_REG3_ADDR, 1);

	tempValue = 0b01000000;	//No reboot, Enable FIFO, Disable HPF, INT1 and OUTPUT directly connected to ADC without filters
	L3GD20_Write(&tempValue, L3GD20_CTRL_REG5_ADDR, 1);

	tempValue = 0b01000000;	//FIFO stream mode, Watermrk level = 0
	L3GD20_Write(&tempValue, L3GD20_FIFO_CTRL_REG_ADDR, 1);
}

//Measure average gyro reading over several seconds, while the sensor is not moved. This is the average bias error, which
//can be subtracted from the raw gyro reading for correction
//This routine measures the bias error for all three axes and all three gyroscope range settings. It saves the 9 values in a global array.
void doGyroBiasCalibration(){
	float avgBuffers[3];
	int32_t readValues[3], counter;
	const uint16_t scales[3]={250, 500, 2000};			//These gyro range settings are calibrated separately
	uint8_t i, scalesToCalibrate;
	for( scalesToCalibrate=0; scalesToCalibrate<=2; scalesToCalibrate++ ){	//For each range setting
		LED_out(0x01<<scalesToCalibrate);
		initSensorGyro( scales[ scalesToCalibrate ] );	//Init range setting, does set the gyroBiasCalValue to the right array
		_setAll3ScalingFactors( gyroBiasCalValue, 0 );	//Set old calibration factors to zero so they do not interfere
		_setAll3ScalingFactors( avgBuffers, 0 );		//Reset accumulating variables
		for( counter=0; counter<=3000; counter++ ){		//Read 3000 samples
			while ( isGyroNewData()==0 ){}				//Wait for new sample
			readSensorGyro( readValues );				//takes 0.05 ms, such wow, SPI much fast!
			for( i=0; i<=2; i++ ){						//For each axis, accumulate samples
				avgBuffers[i] += readValues[i];
			}
		}
		for( i=0; i<=2; i++ ){							//Normalize Accumulated samples to get average
			gyroBiasCalValue[i] = avgBuffers[i] / counter;
		}
	}
	saveCalibrationToFlash();							//Experimental and potentially dangerous: save calibration values to FLASH memory
	initSensorGyro( currentGyroScale );					//Restore original range setting
}

//Experimental and potentially dangerous: save the 3x float arrays with the 3x X,Y,Z calibration values to FLASH memory
void saveCalibrationToFlash(){
	uint32_t flashAdr=FLASH_USER_START_ADDR, *ramAdrs[3];
	uint8_t i,j;
	ramAdrs[0] = (uint32_t*)((void*)gyroBiasCalValue250);
	ramAdrs[1] = (uint32_t*)((void*)gyroBiasCalValue500);
	ramAdrs[2] = (uint32_t*)((void*)gyroBiasCalValue2000);
//	Erase the user Flash area (1 page starting at FLASH_USER_START_ADDR) *********
	FLASH_Unlock();
//	Clear pending flags (if any)
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
//	Erase the FLASH page
	FLASH_ErasePage( FLASH_USER_START_ADDR );
//	Reprogramm FLASH
	FLASH_ProgramWord(flashAdr, 0xDEADBEEF);			//Set the magic word
	flashAdr += 4;
	for( j=0; j<=2; j++ ){
		for( i=0; i<=2; i++ ){
			FLASH_ProgramWord(flashAdr, *ramAdrs[j]++);
			flashAdr += 4;
		}
	}
	FLASH_Lock();
}

//Restore calibration values from flash
void restoreCalibrationFromFlash(){
	uint32_t *calibFlashAdr = (uint32_t*)FLASH_USER_START_ADDR;
	if ( *calibFlashAdr == 0xDEADBEEF ){				//Only do something if the magic word is there
		calibFlashAdr++;
		memcpy( gyroBiasCalValue250, calibFlashAdr, 3*sizeof(float) );
		calibFlashAdr += 3;								//This pointer magic only works because sizeof( float ) == sizeof( uint32_t )
		memcpy( gyroBiasCalValue500, calibFlashAdr, 3*sizeof(float) );
		calibFlashAdr += 3;
		memcpy( gyroBiasCalValue2000, calibFlashAdr, 3*sizeof(float) );
	}
}

//-------------------------------------------------------------------------------------------
// * @brief  Configure the LSM303 Mems compass and accelerometer IC and set scaling registers
//	accelerometerScale is one of the following:     static const UInt16 AccelRangeRamp[] = { 2, 4, 8, 16 };
//	magnetomaterScale  is one of the following:     static const UInt16 MagRangeRamp[]   = { 880, 1300, 1900, 2500 };
//-------------------------------------------------------------------------------------------
void initSensorAccMag( uint8_t accelerometerScale, uint16_t magnetometerScale ) {
	uint8_t tempValue;
	LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
	LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;

	LED_out_byte( accelerometerScale );

    /* Configure the low level I2C interface ---------------------------------------*/
    LSM303DLHC_LowLevel_Init();

	/* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
	LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_ENABLE;
	LSM303DLHC_InitStructure.MagOutput_DataRate = LSM303DLHC_ODR_75_HZ;
	// For an output data rate of 220 Hz, the magnetometer becomes very Glitchy !!!
	switch( magnetometerScale ){
	case 880:
	case 1300:
		LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_1_3_GA;
		_setAll3ScalingFactors( magScalingFactors, MAG_SCALE_FACTOR_1300_XY );
		magScalingFactors[1] = MAG_SCALE_FACTOR_1300_Z;
		break;
	case 1900:
		LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_1_9_GA;
		_setAll3ScalingFactors( magScalingFactors, MAG_SCALE_FACTOR_1900_XY );
		magScalingFactors[1] = MAG_SCALE_FACTOR_1900_Z;
		break;
	case 2500:
	default:
		LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_2_5_GA;
		_setAll3ScalingFactors( magScalingFactors, MAG_SCALE_FACTOR_2500_XY );
		magScalingFactors[1] = MAG_SCALE_FACTOR_2500_Z;
	}
	LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
	LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);

	/* Fill the accelerometer structure */
	LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;		//Not low power
	LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_1344_HZ;
	LSM303DLHCAcc_InitStructure.Axes_Enable = LSM303DLHC_AXES_ENABLE;		//Enable all axes
	switch( accelerometerScale ){
		case 2:
			LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
			_setAll3ScalingFactors( accScalingFactors, ACC_SCALE_FACTOR_2 );
			break;
		case 4:
			LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_4G;
			_setAll3ScalingFactors( accScalingFactors, ACC_SCALE_FACTOR_4 );
			break;
		case 8:
			LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_8G;
			_setAll3ScalingFactors( accScalingFactors, ACC_SCALE_FACTOR_8 );
			break;
		case 16:
		default:
			LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_16G;
			_setAll3ScalingFactors( accScalingFactors, ACC_SCALE_FACTOR_16 );
	}
	LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Single;
	LSM303DLHCAcc_InitStructure.Endianness = LSM303DLHC_BLE_LSB;
	LSM303DLHCAcc_InitStructure.High_Resolution = LSM303DLHC_HR_ENABLE;
	/* Configure the accelerometer main parameters */
	LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);	//Writes to CTRL1 and CTRL4

	tempValue = 0b00010000;	//INT1 Click Off, INT1 AOI1 Off, INT1 AOI2 Off, INT1 DRDY1 ON, INT1 DRDY2 Off, INT1 FIFO watermakr Off, INT1 FIFO overrun OFF
	LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, &tempValue);

	tempValue = 0b01000000;	//No reboot, FIFO On, No Latch interrupt, No 4D (No idea what that is)
	LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, &tempValue);

	tempValue = 0b10000000;	//FIFO mode stream, Trigger INT1?, FTH = 0
	LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_FIFO_CTRL_REG_A, &tempValue);
}

//-------------------------------------------------------------------------------------------
// Returns 1 if the Gyro sensor has data in its FIFO
// The Gyro is configured such that its DRDY / INT2 pin goes high, when its FIFO buffer is empty
// This pin is connected to Pin PE1 on the discovery board. If it is low, there is new data
//-------------------------------------------------------------------------------------------
uint8_t isGyroNewData() {
	return ( GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)==0 );
}

//------------------------------------------------------------------------------------------------
// * @brief  Read 3x 16 bit signed int values over SPI from the gyro
// The result in pfData is scaled such to get a unit of [10^-4 rad/s]
// * @param  pfData : pointer to array[3] where data is written as [X, Y, Z].
// * @returns returns the temperature as int, where one digit corresponds to [0.01 degC]
//-------------------------------------------------------------------------------------------------
int16_t readSensorGyro( int32_t* pfData ) {
	uint8_t buffer[6] = { 0 };
	uint8_t i;
	float tempValueFl;

	L3GD20_Read(buffer, L3GD20_OUT_X_L_ADDR, 6);
	//  Assume fixed Data LSB at lower address!
	for (i=0; i<=2; i++) {
		tempValueFl = (int16_t)( ((uint16_t) buffer[2*i+1] << 8) + buffer[2*i] );//Convert 2 * 8 bit to 32 bit signed integer
		tempValueFl *= gyroScalingFactors[i];									 //Scale to physical units
		tempValueFl -= gyroBiasCalValue[i];										 //Correct bias error (if calibrated)
		pfData[i] = (int32_t)tempValueFl;										 //X, Y, Z
	}

//    Read gyro temperature,
//	  the SDK does the following scaling to get degC:  sensors.Temperature  = s.Temperature * 0.01f;
	L3GD20_Read(buffer, L3GD20_OUT_TEMP_ADDR, 1);
	return( (40 - (int16_t)buffer[0])*100 );	//The conversion factor 40 is mentioned in the datasheet nowhere!
}

//-------------------------------------------------------------------------------------------
// The INT1 pin of the accelerometer is connected to PE4
// The INT2 pin of the accelerometer is connected to PE5
// INT1 is configured as "DRDY1 interrupt on INT1."
// So I guess it goes high when Data is ready
//-------------------------------------------------------------------------------------------
uint8_t isAccNewData() {
	return ( GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) );
}

//-------------------------------------------------------------------------------------------
// Read acceleration from Sensor,
// The result in pfData is scaled such to get a unit of [10^-4 m/s^2]
// * @param pnData: pointer to int buffer array[3] where to store data as [X, Y, Z]
//-------------------------------------------------------------------------------------------
void readSensorAcc( int32_t* pfData ) {
	uint8_t buffer[6];
	float tempValueFl;
	uint8_t i = 0;
//  float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;

	/* Read the 12 bit data in the buffer*/
	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

	//Assume little endian mode!
	for (i=0; i<=2; i++) {
		tempValueFl = (int16_t)( ((uint16_t) buffer[2*i+1] << 8) + buffer[2*i] );//Convert 2 * 8 bit to 32 bit signed integer
		tempValueFl *= accScalingFactors[i];									 //Scale to physical units [10^-4 m/s^2]
		pfData[i] = (int32_t)tempValueFl;										 //X, Y, Z
	}
}

//-------------------------------------------------------------------------------------------
// Returns 1 if the Magnetometer sensor actually has new data to read
//-------------------------------------------------------------------------------------------
uint8_t isMagNewData() {
	return GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
}

//-------------------------------------------------------------------------------------------
// * Read LSM303DLHC output register, to get the magnetic field vectors,
// * The data is read from sensor as 16 bit signed integers (only the 12 lower order bits are used)
// 	min: -2048 (0xF800)   max: +2047 (0x07FF)
// The result in pfData is scaled such to get a unit of [10^-4 Gauss]
// * pfData[0] = MagX  pfData[1] = MagZ  pfData[2] = MagY
// * the temperature is returned (as signed int)  but not implemented yet
//-------------------------------------------------------------------------------------------
int16_t readSensorMag( int16_t* pfData ) {
	uint8_t buffer[6];
	uint8_t i = 0;
	float tempValueFl;
	/* Read the 12 bit mag vector data in the buffer*/
	LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 6);
	//Assume little endian mode!
	for (i=0; i<=2; i++) {
		//Unscaled 16 bit value (only 12 valid bits on LSB)
		tempValueFl = (int16_t)( ((uint16_t) buffer[2*i] << 8) | buffer[2*i+1] );
		tempValueFl *= magScalingFactors[i];									 //Scale to physical units [10^-4 Gauss]
		pfData[i] = tempValueFl;												 //X, Z, Y
	}
	/* Read the 12 bit temp value in buffer*/
//	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_TEMP_OUT_H_M, buffer, 2);
//	TODO: the two bytes for the temperature register are always read 0, why?
//	tempValue = (int16_t)(((uint16_t) buffer[0] << 8) | buffer[1] );
	return( 0 );
}



// -----------------------------------------------------------------------
// *  What follows is LED and button stuff                               *
// -----------------------------------------------------------------------
/**
 * Initialize the LEDS on the Discovery board
 */
void LED_Config(void) {
	uint8_t i;
	for (i = LED3; i <= LED10; i++) {
		STM_EVAL_LEDInit(i);
	}
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
}

//Display a byte on the 8 LEDs, with LSB on LD3 and MSB on LD10 (as written on the PCB silkscreen)
void LED_out_byte( uint8_t dat ){
	uint8_t i, tmp=1;
	for( i=0; i<=7; i++){
		if( dat & tmp ){
			STM_EVAL_LEDOn( i );
		} else {
			STM_EVAL_LEDOff( i );
		}
		tmp<<=1;
	}
}

//Display a LED animation, when the tracker is running
void LED_ani_run(){
	static uint32_t loopCounter;
	static uint8_t state=0;
	if (loopCounter++%1000){
		return;
	}
	state<<=1;
	state |= (state&0x80)==0;
	LED_out( state );
}

// Do PWM on the LED ports to show some nice patterns during IDLE
void LED_ani_idle(){
	static uint16_t wait=0;
	static uint8_t pwmCount=0;
	static uint8_t BRIGHTNESS[] = {0,0,0,0,0,0,0,0,0,0,1,8,13,23,38,64,38,23,13,8,1,0,0,0};	//Brightness values shifted through the 8 LEDs (values between 0-64)
	const static uint8_t BRIGHTNESS_LENGTH = 24;				//Length of the bightness array
	uint8_t tmpOut=0, portPinMask, portPin, temp, temp2;
	if(--wait==0){												//Every _wait_ cycles, rotate the BRIGHTNESS array by one
		wait = 25000;
		temp2 = BRIGHTNESS[ 0 ];								//Save lowest element
		for( temp=0; temp<BRIGHTNESS_LENGTH-1; temp++){
			BRIGHTNESS[ temp ] = BRIGHTNESS[ temp+1 ];			//Move all elements one step down
		}
		BRIGHTNESS[ BRIGHTNESS_LENGTH-1 ] = temp2;				//Put first element on top
	}
	portPinMask = 1;											//Shift a one through the portPinMask register, to set individual portPins
	for(portPin=0; portPin<=7; portPin++){						//for all portPins:
		if( BRIGHTNESS[portPin] > pwmCount )					//	check if it is time to set a specific portPin
			tmpOut |= portPinMask;								//	the larger the BRIGHTNESS value, the sooner the pin will be set to one
		portPinMask<<=1;
	}
	if( ++pwmCount >= 64 ){
		pwmCount = 0;
	}
	LED_out( tmpOut );
}

//Display a byte on the 8 LEDs, where it goes around circles
void LED_out( uint8_t dat ){
	GPIOE->ODR = (GPIOE->ODR&0x0F) | (dat<<8);
}

//Switch off all 8 LEDs
void LEDs_Off(void) {
	uint8_t i;
	for (i = LED3; i <= LED10; i++) {
		STM_EVAL_LEDOff(i);
	}
}

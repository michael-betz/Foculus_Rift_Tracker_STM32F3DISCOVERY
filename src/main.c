/**
 ******************************************************************************
 * @file    main.c
 * @author  YFL
 * @version V0.4
 * @date    11.5.2014
 * @brief   Main program body
 ******************************************************************************
 */

// The "Foculus Rift" tracker
//------------------------------------------------------------------
// A USB HID device, sending Accelerometer, Gyroscope and Magnetometer data over USB in an
// Oculus Rift compatible format.
// This project comes in useful, if you are building your own DIY Head Mounted Display.
// It solves the problem of finding a motion tracker.
// The firmware runs on the STM32F3DISCOVERY board and should be compatible with all Oculus Rift games.

// development blog:   http://yetifrisstlama.blogspot.fr/2014/03/the-foculus-rift-part-2-reverse.html

// Organization:
//------------------------------------------------------------------
// main.c   		main logic of the tracker,
//                  handling and reformating the sensor data stream and packing it into 62 byte packets to be sent over USB,
//                  Assign the coordinate system directions
//                  keeping track of the configuration data structures which the libOVR might send and request
// peripherals.c    setup and request data from the 3 sensor chips over SPI and I2C
//                  Handle the scaling and calibration factors, so the headtracker moves in the right way
//                  Zero-level calibration routine for the gyroscope
//                  Save calibration factors to flash
//                  PWM-patterns for the 8 status LEDs
// usb_desc.c       USB - HID descriptors, which fool the PC into thinking that there is an Oculus RIft corrected
// usb_endp.c       STM USB driver endpoint1 callbacks, just sets some global flags to inform the main routine when there is new data
// usb_prop.c       Customization of the STM USB driver, so feature reports can be sent and received
//                  received data is copied in the global array featureReportData[] and then processed by the main loop


// THings to implement
//------------------------------------------------------------------
// * Evaluate the flags, I still do not have a clue what they are used for
// * What is this mysterious 16 bit command field in the Oculus packets for?
// * FixMe: There is this strange synchronization issue, where featureReportData[] sometimes contains invalid data when evaluated in main()


// Changelog
//--------------------------------
// 09.03.2014:	Fixed bug in handleConfigPacketEvent() by adding break; statements (data rate was always 1 ms before)
// 10.03.2014:  Changed wMaxPacketSize from 62 to 64
// 19.03.2014:  Changed I2C Bus speed to 400 kHz, which allows to read all 3 sensors in 0.65 ms  (before it was > 2 ms)
// 20.03.2014:  Now evaluating the "new data ready" pins of all 3 sensors (improves timing a lot, reduces jitter)
//              Enabled FIFO in Streaming mode of Accelerometer and Gyro (no samples will be lost!)
//              Fixed Glitches in Magnetometer output by setting it to 75 Hz measurement rate (was 220 Hz before)
// 23.03.2014:  Fixed a problem with the USB interrupt and atomic access, not allowing the tracker to change sensor scale
//				Changed sensor scaling to floating point numbers and scaled to values as expected from the SDK
// 29.03.2014:  Added gyroscope "set to zero" calibration routine (Press the user button on the STM board and keep it very still)
//				Added temperature readout from gyro
//				Added some nice LED animations for IDLE mode, Tracker running mode and Calibration mode
// 01.04.2014:	Gyro offset calibration is now saved to Flash at address 0x08006000 and hence retained after power off
// 08.05.2014:	Fixed bug in readSensorAcc(): an array was accessed outside its boundaries.
//				also included the .hex file and switched on compiler optimizations
// 11.05.2014:	Experimental: Setting of board orientation ...
//				Push the user button for > 1 s to choose between 8 preconfigured orientation settings
//				Push the user button again for > 1 s to save the setting to FLASH memory

#pragma pack(1)		//If this is not defined, GCC Will use padding bytes and mess up the union structs

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "main.h"
#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include "globalVariables.h"
#include "peripherals.h"

/* Init global variables -----------------------------------------------------*/
RiftDisplayInfoFrameSep dataDisplayInfo;
RiftKeepAliveFrameSep dataKeepAlive;
RiftConfigFrameSep dataConfig;
RiftScaleFrameSep dataScale;
RiftSensorFrameSep dataToBeSent;
__IO uint8_t EP1flagTXcomplete = 1;
__IO uint8_t EP1flagRXcomplete = 0;
uint8_t orientationIndex=0;						//Holds the current orientation setting of the coordinate system (0-7)
												//This variable is saved to flash and restored on power up
												//Setup the corresponding orientations in packFoculusdataBlock()

/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;
// Timer compare values are buffered locally and updated in the configFrame received event
uint32_t currentKeepAliveTimeout = 10000;		//[ms]
uint16_t currentDelayBetweenPackets = 100;		//[ms]
uint8_t  EP1debugBuffer[128];					//Not really used at the moment
int32_t gyroBuffer[3];							//[X,Y,Z] Will hold the scaled and ready to send sensor data
int32_t accBuffer[3];							//[X,Y,Z]
int16_t magBuffer[3];							//[X,Z,Y]
int16_t tempBuffer;								//Holds temperature as read from Magnetometer

/* Private function prototypes -----------------------------------------------*/
void initDefaultRiftSettings();
void packFoculusdataBlock();
void packSensorDataBlock(int32_t Xin, int32_t Yin, int32_t Zin, uint8_t *sensorDataOut);
void handleConfigPacketEvent();

enum FoculusStates {
    FocSt_Init,			// Pause sending data (keep alive timeout for example)
    FocSt_SendData		// Send data
};
uint8_t focState = FocSt_Init;


//Implement the user interface in one big ugly blocking function.
//I'm fully aware that this does stall the USB communications, which might cause trouble!
void handleButton(){
	uint8_t i;
	LED_out_byte( 0x00 );						//See how long ...
	i = getShortLongButtonPress();
	if( i==1 ){									//Short button press
		for (i=0; i<=5; i++){					//Blink LEDs 5 times to indicate gyro bias calibration
			delayms( 100 );
			LED_out_byte( 0xFF );
			delayms( 100 );
			LED_out_byte( 0x00 );
		}
		doGyroBiasCalibration();				//Start calibration measurement
	} else if( i==2 ){							//Long button press, do selection of orientation
		STM_EVAL_LEDOn( orientationIndex );
		while( STM_EVAL_PBGetState( BUTTON_USER ) ); //Wait for button release
		while( 1 ){
			STM_EVAL_LEDOn( orientationIndex );	//Indicate current orientation index
			i = getShortLongButtonPress();
			if( i==1 ){							//If short press, increment orientation index
				orientationIndex++;
				if( orientationIndex >= 8 ){
					orientationIndex = 0;
				}
				LED_out_byte( 0x00 );
			} else if ( i==2 ){					//If long press, save and exit orientation mode
				saveCalibrationToFlash();		//Experimental and potentially dangerous: save calibration values and orientation to FLASH memory
				for (i=0; i<=5; i++){			//Blink LEDs 5 times to indicate save
					delayms( 100 );
					STM_EVAL_LEDOn( orientationIndex );
					delayms( 100 );
					LED_out_byte( 0x00 );
				}
				break;
			}
		}
	}
}

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void) {
	uint8_t i;	//Generic for loop counter variable
	/* Setup SysTick event each 1 ms */
	RCC_GetClocksFreq( &RCC_Clocks );
	SysTick_Config( RCC_Clocks.HCLK_Frequency / 1000 );	  //Every 1 ms

	/* Configure the USB (will get stuck here until USB is plugged in)*/
	USB_Config();

	/* Init peripherals */
	LED_Config();
	initDefaultRiftSettings();
	initSensorGyro( dataScale.rFrame.gScale );
	initSensorAccMag( dataScale.rFrame.aScale, dataScale.rFrame.mScale );	// Init combined accelerometer and magnetometer chip

	/* Infinite loop */
	while (1) {
//---------------------------------------------------------------------------------------------------------
//		Is the reception of a new packet on Endpoint 1 (EP1) finished? This is not used at the moment.
//---------------------------------------------------------------------------------------------------------
		if ( EP1flagRXcomplete ) {
			USB_SIL_Read( EP1_OUT, EP1debugBuffer );
			SetEPRxValid( ENDP1 );
			EP1flagRXcomplete = 0;
//			uint8_t command = EP1debugBuffer[0];
		}
//----------------------------------------------------------------------------------------------------
//		Check if some settings have been updated by the USB interrupt and fire an event to handle the setting change
//----------------------------------------------------------------------------------------------------
		if ( newConfigFrameReceived >= 0 ){
			if ( newConfigFrameReceived == featureReportData[0] ){	//Rudimentary check if featureReportData contains valid data (it does not always and I don't understand why)
				handleConfigPacketEvent();
			} else {
				newConfigFrameReceived = -1;						//Sometimes a glitch happens and no new data is written to featureReportData
			}
		}
//----------------------------------------------------------------------------------------------------
//		Check if user pushed the button to change the settings
//----------------------------------------------------------------------------------------------------
		if ( STM_EVAL_PBGetState( BUTTON_USER ) ){	//Check if button is pressed
			handleButton();							//ToDo: !!! Danger !!! This is blocking and will stall USB communications!
		}
//----------------------------------------------------------------------------------------------------
//		Handle sensor data statemachine
//----------------------------------------------------------------------------------------------------
		switch ( focState ){
		case FocSt_Init:									// Do not send sensor data while in init state, wait for keepALive packet
			LED_ani_idle();
			break;
		case FocSt_SendData:
			LED_ani_run();
			if ( isGyroNewData() ){
				tempBuffer = readSensorGyro( gyroBuffer );	//takes 0.05 ms, wow, SPI, such fast!
			}
			if ( isAccNewData() ){
				readSensorAcc( accBuffer );					//takes 0.23 ms
			}
			if ( isMagNewData() ){
				readSensorMag( magBuffer );					//takes 0.37 ms
			}
			// Check if a new packet shall be sent over USB
			// Do this only if: _last transmission finished_  and if _enough time passed_
			if ( EP1flagTXcomplete && (timerTimeSinceLastFrameSent >= currentDelayBetweenPackets) ) {
				EP1flagTXcomplete = 0;			//Yep, start new USB transmission ...
				timerTimeSinceLastFrameSent = 0;
				packFoculusdataBlock();			//Format the data from sensors as a SensorFrame
				USB_SIL_Write( EP1_IN, dataToBeSent.rawUSBdata, 62 );	//Hand data over to the USB driver
				SetEPTxValid( ENDP1 );
			}
			//  * SKD sends keep alive packets every 3 seconds or so, they contain keepALiveIntervalMs
			//	  this is a (10 s) timeout, if this time has passed and no keep alive packet received, then
			//    stop sending sensor data.
			// Check if a keepAlive timout occurred
			if ( timerTimeSinceLastKeepAlive > currentKeepAliveTimeout ){
				focState = FocSt_Init;			//Timeout occurred, back to init state
			}
			break;
		}
	}
}

// Copy data form the memory buffer used by the USB interrupt
// to the structs used by the main routine, which are useful for decoding purposes
// It is a bit tricky to access data from an interrupt routine, as it can change anytime!
void memcpy_atomic(uint8_t *to, __IO uint8_t *from, size_t size){
	size_t counter;
	for (counter=0; counter<size; counter++){
		*to = *from;
		to++;
		from++;
	}
//	memcpy(to, from, size);		//Not sure if the above (using volatile) is any better (or even worse) than this
}

void handleConfigPacketEvent(){
// This is called, after a new ConfigPacket has been received from the libOVR.
// At this point, the new configuration data needs to be copied in the relevant struct
// Then we react accordingly to the updated configuration
	uint16_t command;
	switch ( newConfigFrameReceived ){
	case Packet_KeepAlive:		//		Reset keepAlive Counter, set state to Run
		memcpy_atomic( dataKeepAlive.rawUSBdata, featureReportData, 5 );
		newConfigFrameReceived = -1;
		if ( dataKeepAlive.rFrame.keepAliveIntervalMs > 100 ){
			currentKeepAliveTimeout = dataKeepAlive.rFrame.keepAliveIntervalMs;
		}
		if ( focState==FocSt_Init ){
			focState = FocSt_SendData;
		}
		command = dataKeepAlive.rFrame.commandID;
		break;
	case Packet_Config:			//		Reset packet interval, keep ALive Interval and handle Flags
		memcpy_atomic( dataConfig.rawUSBdata, featureReportData, 7 );
		newConfigFrameReceived = -1;
		currentDelayBetweenPackets = dataConfig.rFrame.packetInterval;
		if ( dataConfig.rFrame.keepAliveIntervalMs > 100 ){
			currentKeepAliveTimeout = dataConfig.rFrame.keepAliveIntervalMs;
		}
		command = dataConfig.rFrame.commandID;
		break;
	case Packet_DisplayInfo:	//		Note, that at the moment this allows write access to the DisplayInfo data
		memcpy_atomic( dataDisplayInfo.rawUSBdata, featureReportData, 56 );
		newConfigFrameReceived = -1;
		command = dataDisplayInfo.rFrame.commandID;
		break;
	case Packet_Scale:			//		Reset Sensor chips and scaling factors to new scale
		memcpy_atomic( dataScale.rawUSBdata, featureReportData, 8 );
		newConfigFrameReceived = -1;
		initSensorGyro( dataScale.rFrame.gScale );
		initSensorAccMag( dataScale.rFrame.aScale, dataScale.rFrame.mScale );
		command = dataScale.rFrame.commandID;
		break;
	default:
		newConfigFrameReceived = -1;
	}
	timerTimeSinceLastKeepAlive = 0;
}

/**
 * @brief  Configure the USB.
 */
void USB_Config(void) {
	Set_System();
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
	while (bDeviceState != CONFIGURED) {
	}
}

//The SDK will request some configuration data from the tracker, here I set the default setting
//I tried to find sensible default settings, but especially for the displayInfo struct I'm not so sure if they are OK.
void initDefaultRiftSettings() {
	dataConfig.rFrame.commandID = 0;
	dataConfig.rFrame.flags = Flag_RawMode | Flag_SensorCoordinates;
	dataConfig.rFrame.keepAliveIntervalMs = currentKeepAliveTimeout;
	dataConfig.rFrame.packetInterval = currentDelayBetweenPackets;
	dataConfig.rFrame.two = 0x02;

	dataScale.rFrame.commandID = 0;
	dataScale.rFrame.aScale = 8;
	dataScale.rFrame.gScale = 1000;
	dataScale.rFrame.mScale = 2500;
	dataScale.rFrame.four = 0x04;

	dataKeepAlive.rFrame.commandID = 0;
	dataKeepAlive.rFrame.keepAliveIntervalMs = currentKeepAliveTimeout;
	dataKeepAlive.rFrame.eight = 0x08;

	dataDisplayInfo.rFrame.commandID = 0;
	dataDisplayInfo.rFrame.distortionType =  0x00;		//ToDo: Distortion Type Flags, What is possible here?
	dataDisplayInfo.rFrame.HResolution = 	 1280;
	dataDisplayInfo.rFrame.VResolution = 	  800;
	dataDisplayInfo.rFrame.HScreenSize =   149760;		//[um] ?
	dataDisplayInfo.rFrame.VScreenSize =  	93600;		//[um] ?
	dataDisplayInfo.rFrame.VCenter = 		46900;		//[um] ?
	dataDisplayInfo.rFrame.LensSeparation = 64000;		//[um] ?
	dataDisplayInfo.rFrame.eye2ScreenDistanceL = 41000; //[um] ?
	dataDisplayInfo.rFrame.eye2ScreenDistanceR = 41000; //[um] ?
	dataDisplayInfo.rFrame.k0 = 1.00;
	dataDisplayInfo.rFrame.k1 = 0.22;
	dataDisplayInfo.rFrame.k2 = 0.24;
	dataDisplayInfo.rFrame.k3 = 0.00;
	dataDisplayInfo.rFrame.k4 = 0.00;
	dataDisplayInfo.rFrame.k5 = 0.00;
	dataDisplayInfo.rFrame.nine = 0x09;

	dataToBeSent.rFrame.messageType = TrackerMessage_Sensors;	//This means we send sensor data
	dataToBeSent.rFrame.sampleCount = 0;
	dataToBeSent.rFrame.timeStamp = 0;
	dataToBeSent.rFrame.commandID = 0;
	dataToBeSent.rFrame.temp = 4711;
	dataToBeSent.rFrame.mX = 4;
	dataToBeSent.rFrame.mY = 5;
	dataToBeSent.rFrame.mZ = 6;

	restoreCalibrationFromFlash();						//Restore gyro offset calibration factors once at startup from flash
}

//**********************************************************************************
//* Functions for fetching data from the Sensors and formating them in RIFT format *
//**********************************************************************************
void packSensorDataBlock( int32_t Xin, int32_t Yin, int32_t Zin, uint8_t *sensorDataOut ) {
/*
 * Within the readSensor() functions:
 *  the 16 bit int sensor values from the Gyro and Acc are scaled to physical units and cast to int32_t
 * Here:
 *  The first 21 LSB bits are used in this routine and reformated to fit in the dataBlock as expected by the Oculus SDK
 *  21 bit effective input range:     -1 048 576 (0x100000)   ...    -1 (0x1FFFFF)   ...    +1 048 575  (0x0FFFFF)
 */
	//Arrange the bits in the right order within the buffer
	sensorDataOut[0] = (Xin >> 13) & 0xFF;							//Xin[13:20]  (bit index 13 - 20 of Xin)
	sensorDataOut[1] = (Xin >>  5) & 0xFF;							//Xin[5:12]
	sensorDataOut[2] = ((Xin & 0x1F) << 3) | ((Yin >> 18) & 0x07);	//Xin[0:4],Yin[18:20] (MSB, part of Xin, part of Yin, LSB)

	sensorDataOut[3] = (Yin >> 10) & 0xFF;							//Yin[10:17]
	sensorDataOut[4] = (Yin >>  2) & 0xFF;							//Yin[2:9]
	sensorDataOut[5] = ((Yin & 0x03) << 6) | ((Zin >> 15) & 0x3F);	//Yin[0:1],Zin[15:20]

	sensorDataOut[6] = (Zin >>  7) & 0xFF;							//Zin[7:14]
	sensorDataOut[7] = (Zin <<  1) & 0xFE;							//Zin[0:6],0
}

void packFoculusdataBlock() {
//	Here we do the transformation from the two different sensor coordinate systems
//	to the libOVR coordinate system. We also allow 8 different mounting orientations
//	of the STM discovery board.
//  If the coordinate axes seem not right, here is the place to change it
	static uint16_t timeStamp = 0;
	dataToBeSent.rFrame.temp = tempBuffer;																		//Temperature
	dataToBeSent.rFrame.commandID = 0;
	dataToBeSent.rFrame.sampleCount = 1;
	dataToBeSent.rFrame.timeStamp = timeStamp++;
	switch( orientationIndex ){		//This allows the sensor board to be mounted in 8 different orientations (configurable by the Userbutton)
	default:
	case 0:	// These settings seem to fit for mounting the discovery board on the back of the HMD,
			// so that the LEDs point toward the face and the USB connectors point downwards
			// See also: riftCoord.png
		dataToBeSent.rFrame.mX = -magBuffer[ cmY ];
		dataToBeSent.rFrame.mY =  magBuffer[ cmX ];
		dataToBeSent.rFrame.mZ =  magBuffer[ cmZ ];
		packSensorDataBlock(  -accBuffer[ caY ],  accBuffer[ caX ],  accBuffer[ caZ ], dataToBeSent.rFrame.sensorData1 );		//Conv. Acc. data to Oculus format
		packSensorDataBlock(  -gyroBuffer[ cgX ], -gyroBuffer[ cgY ], gyroBuffer[ cgZ ], dataToBeSent.rFrame.sensorData1+8 );	//Conv. Gyro. data to Oculus format
		break;
	case 1:
		dataToBeSent.rFrame.mX =  magBuffer[ cmY ];
		dataToBeSent.rFrame.mY = -magBuffer[ cmX ];
		dataToBeSent.rFrame.mZ =  magBuffer[ cmZ ];
		packSensorDataBlock(  accBuffer[ caY ],  -accBuffer[ caX ],  accBuffer[ caZ ], dataToBeSent.rFrame.sensorData1 );		//Conv. Acc. data to Oculus format
		packSensorDataBlock(  gyroBuffer[ cgX ], gyroBuffer[ cgY ], gyroBuffer[ cgZ ], dataToBeSent.rFrame.sensorData1+8 );	//Conv. Gyro. data to Oculus format
		break;
	case 2:
		dataToBeSent.rFrame.mX =  magBuffer[ cmY ];
		dataToBeSent.rFrame.mY =  magBuffer[ cmX ];
		dataToBeSent.rFrame.mZ = -magBuffer[ cmZ ];
		packSensorDataBlock(  accBuffer[ caY ],  accBuffer[ caX ],  -accBuffer[ caZ ], dataToBeSent.rFrame.sensorData1 );		//Conv. Acc. data to Oculus format
		packSensorDataBlock(  gyroBuffer[ cgX ], -gyroBuffer[ cgY ], -gyroBuffer[ cgZ ], dataToBeSent.rFrame.sensorData1+8 );	//Conv. Gyro. data to Oculus format
		break;
	case 3:
		dataToBeSent.rFrame.mX = -magBuffer[ cmY ];
		dataToBeSent.rFrame.mY = -magBuffer[ cmX ];
		dataToBeSent.rFrame.mZ = -magBuffer[ cmZ ];
		packSensorDataBlock(  -accBuffer[ caY ],  -accBuffer[ caX ],  -accBuffer[ caZ ], dataToBeSent.rFrame.sensorData1 );		//Conv. Acc. data to Oculus format
		packSensorDataBlock(  -gyroBuffer[ cgX ], gyroBuffer[ cgY ], -gyroBuffer[ cgZ ], dataToBeSent.rFrame.sensorData1+8 );	//Conv. Gyro. data to Oculus format
		break;
	case 4:
		dataToBeSent.rFrame.mX = -magBuffer[ cmY ];
		dataToBeSent.rFrame.mY =  magBuffer[ cmZ ];
		dataToBeSent.rFrame.mZ = -magBuffer[ cmX ];
		packSensorDataBlock(  -accBuffer[ caY ],  accBuffer[ caZ ],  -accBuffer[ caX ], dataToBeSent.rFrame.sensorData1 );		//Conv. Acc. data to Oculus format
		packSensorDataBlock(  -gyroBuffer[ cgX ], gyroBuffer[ cgZ ], gyroBuffer[ cgY ], dataToBeSent.rFrame.sensorData1+8 );	//Conv. Gyro. data to Oculus format
		break;
	case 5:
		dataToBeSent.rFrame.mX = magBuffer[ cmY ];
		dataToBeSent.rFrame.mY = magBuffer[ cmZ ];
		dataToBeSent.rFrame.mZ = magBuffer[ cmX ];
		packSensorDataBlock(  accBuffer[ caY ],  accBuffer[ caZ ],  accBuffer[ caX ], dataToBeSent.rFrame.sensorData1 );		//Conv. Acc. data to Oculus format
		packSensorDataBlock(  gyroBuffer[ cgX ], gyroBuffer[ cgZ ], -gyroBuffer[ cgY ], dataToBeSent.rFrame.sensorData1+8 );	//Conv. Gyro. data to Oculus format
		break;
	case 6:
		dataToBeSent.rFrame.mX = -magBuffer[ cmX ];
		dataToBeSent.rFrame.mY =  magBuffer[ cmZ ];
		dataToBeSent.rFrame.mZ =  magBuffer[ cmY ];
		packSensorDataBlock(  -accBuffer[ caX ],  accBuffer[ caZ ],  accBuffer[ caY ], dataToBeSent.rFrame.sensorData1 );		//Conv. Acc. data to Oculus format
		packSensorDataBlock(  gyroBuffer[ cgY ], gyroBuffer[ cgZ ], gyroBuffer[ cgX ], dataToBeSent.rFrame.sensorData1+8 );	//Conv. Gyro. data to Oculus format
		break;
	case 7:
		dataToBeSent.rFrame.mX =  magBuffer[ cmX ];
		dataToBeSent.rFrame.mY =  magBuffer[ cmZ ];
		dataToBeSent.rFrame.mZ = -magBuffer[ cmY ];
		packSensorDataBlock(  accBuffer[ caX ],  accBuffer[ caZ ],  -accBuffer[ caY ], dataToBeSent.rFrame.sensorData1 );		//Conv. Acc. data to Oculus format
		packSensorDataBlock(  -gyroBuffer[ cgY ], gyroBuffer[ cgZ ], -gyroBuffer[ cgX ], dataToBeSent.rFrame.sensorData1+8 );	//Conv. Gyro. data to Oculus format
		break;
	}
}

/**
 * @brief  Basic management of the timeout situation.
 * @param  None.
 * @retval None.
 */

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

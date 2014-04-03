/*
 * foculusTypes.h
 *
 *  Created on: Dec 15, 2013
 *      Author: michael
 */

#ifndef FOCULUSTYPES_H_
#define FOCULUSTYPES_H_

/* ------------------------------------------------------------------------- */
/* ----------------------- FOculus Constants  ------------------------------ */
/* ------------------------------------------------------------------------- */
// Configuration Packet Types (Sent over EP0 by feature reports)
#define Packet_Config			0x02
#define	Packet_Scale			0x04
#define Packet_KeepAlive		0x08
#define Packet_DisplayInfo		0x09

// Flag values for Config packet.
#define Flag_RawMode 			0x01
#define Flag_CalibrationTest	0x02	// Internal test mode
#define Flag_UseCalibration		0x04
#define Flag_AutoCalibration	0x08
#define Flag_MotionKeepAlive	0x10
#define Flag_CommandKeepAlive	0x20
#define Flag_SensorCoordinates	0x40

enum TrackerMessageType {
    TrackerMessage_None              = 0,
    TrackerMessage_Sensors           = 1,
    TrackerMessage_Unknown           = 0x100,
    TrackerMessage_SizeError         = 0x101,
};

/* ------------------------------------------------------------------------- */
/* ----------------------- FOculus data types ------------------------------ */
/* ------------------------------------------------------------------------- */
// Describes one complete dataset which will be sent over USB to the PC
typedef struct RiftSensorFrame{
	uint8_t messageType;
	uint8_t sampleCount;		// How many snensorData slots are used (1, 2 or 3)
	uint16_t timeStamp;
	uint16_t commandID;
	int16_t temp;
	uint8_t sensorData1[16];	// 3 sensor data packet, 16 bytes each.
	uint8_t sensorData2[16];	// 3 sensor data packet, 16 bytes each.
	uint8_t sensorData3[16];	// 3 sensor data packet, 16 bytes each.
	int16_t mX;				// Magnetometer data
	int16_t mY;
	int16_t mZ;
}RiftSensorFrame;
typedef union RiftSensorFrameSep {			//This we use to get access to data on binary and C-variable level
	uint8_t rawUSBdata[62];
	RiftSensorFrame rFrame;	//SepeHoldsrates header, magnetometer and raw sensor data
}RiftSensorFrameSep;

// Describes the sensor scaling
//static const UInt16 AccelRangeRamp[] = { 2, 4, 8, 16 };
//static const UInt16 GyroRangeRamp[]  = { 250, 500, 1000, 2000 };
//static const UInt16 MagRangeRamp[]   = { 880, 1300, 1900, 2500 };
typedef struct RiftScaleFrame{
	uint8_t four;		//USB HID Report ID
	uint16_t commandID;
	uint8_t  aScale;	//Accelerometer Scale
	uint16_t gScale;	//Gyroscope scale
	uint16_t mScale;	//Magnetometer scale
}RiftScaleFrame;
typedef union RiftScaleFrameSep{		//This we use to get access to data on binary and C-variable level
	uint8_t rawUSBdata[8];
	RiftScaleFrame rFrame;		//SepeHoldsrates header, magnetometer and raw sensor data
}RiftScaleFrameSep;

// Describes the config flags
typedef struct RiftConfigFrame{
	uint8_t  two;		//USB HID Report ID
	uint16_t commandID;
	uint8_t  flags;
	uint8_t  packetInterval;
	uint16_t keepAliveIntervalMs;
}RiftConfigFrame;
typedef union RiftConfigFrameSep{			//This we use to get access to data on binary and C-variable level
	uint8_t rawUSBdata[7];
	RiftConfigFrame rFrame;					//SepeHoldsrates header, magnetometer and raw sensor data
}RiftConfigFrameSep;

// Describes the KeepAlive packet
typedef struct RiftKeepAliveFrame{
	uint8_t  eight;				//USB HID Report ID
	uint16_t commandID;
	uint16_t keepAliveIntervalMs;
}RiftKeepAliveFrame;
typedef union RiftKeepAliveFrameSep{//This we use to get access to data on binary and C-variable level
	uint8_t rawUSBdata[5];
	RiftKeepAliveFrame rFrame;	//SepeHoldsrates header, magnetometer and raw sensor data
}RiftKeepAliveFrameSep;

// Describes the DisplayInfo packet
typedef struct RiftDisplayInfoFrame{
	uint8_t  nine;			//USB HID Report ID
	uint16_t commandID;
	uint8_t	 distortionType;
	uint16_t HResolution;
	uint16_t VResolution;
	uint32_t HScreenSize;
	uint32_t VScreenSize;
	uint32_t VCenter;
	uint32_t LensSeparation;
	uint32_t eye2ScreenDistanceL;
	uint32_t eye2ScreenDistanceR;
	float   k0;						//Lens distortion coefficients
	float   k1;
	float   k2;
	float   k3;
	float   k4;
	float   k5;
}RiftDisplayInfoFrame;
typedef union RiftDisplayInfoFrameSep{			//This we use to get access to data on binary and C-variable level
	uint8_t rawUSBdata[56];
	RiftDisplayInfoFrame rFrame;	//SepeHoldsrates header, magnetometer and raw sensor data
}RiftDisplayInfoFrameSep;

#endif /* FOCULUSTYPES_H_ */

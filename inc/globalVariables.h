#ifndef GLOBALVAR_H_
#define GLOBALVAR_H_
#include <stdint.h>
#include "foculusTypes.h"

//---------------------------------------------------------------------------------------
// Array which holds data received as HID feature report (in an interrupt routine, thats why volatile)
//---------------------------------------------------------------------------------------
extern __IO uint8_t featureReportData[128];

//---------------------------------------------------------------------------------------
// Rift configuration frames, which hold the current config data
// they are Get and Set by the USB endpoint 0
//---------------------------------------------------------------------------------------
extern RiftDisplayInfoFrameSep	dataDisplayInfo;
extern RiftKeepAliveFrameSep	dataKeepAlive;
extern RiftConfigFrameSep 		dataConfig;
extern RiftScaleFrameSep		dataScale;
extern RiftSensorFrameSep		dataToBeSent;	//Next sensor frame to transmit

//---------------------------------------------------------------------------------------
// Interrupt variables, for synchronizing with the main routine
// From USB and timer
//---------------------------------------------------------------------------------------
extern __IO uint8_t EP1flagTXcomplete;				//Sending of data complete (IN1 interrupt endpoint)
extern __IO uint8_t EP1flagRXcomplete;				//Receiving of data complete (OUT1 interrupt endpoint)
extern __IO int8_t  newConfigFrameReceived;			//If >= 0 then a new config frame has been received on EP0, the number is the ID, used to fire an event in main
extern __IO uint32_t timerTimeSinceLastKeepAlive;	//Keep alive countdown, every ms, stops at zero
extern __IO uint32_t timerTimeSinceLastFrameSent;	//Counts up every ms.
extern __IO uint32_t timerTimeSinceLastAccMagReadout;

//---------------------------------------------------------------------------------------
// Global functions
//---------------------------------------------------------------------------------------


#endif

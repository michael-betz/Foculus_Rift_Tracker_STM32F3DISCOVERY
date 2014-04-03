Foculus_Rift_Tracker_STM32F3DISCOVERY
=====================================

// The "Foculus Rift" tracker
//-------------------------------
// A USB HID device, sending Accelerometer, Gyroscope and Magnetometer data over USB in a
// Rift compatible format

// THings to implement
//-------------------------------
// * Evaluate the flags, I still do not have a clue what they are used for
// * What is this mysterious 16 bit command field in the Oculus packets for?
// * Why is there still no good yaw drift correction?
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

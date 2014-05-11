The "Foculus Rift" tracker
==========================
A USB HID device, sending Accelerometer, Gyroscope and Magnetometer data over USB in an Oculus Rift compatible format.

This project comes in useful, if you are building your own DIY Head Mounted Display. It solves the problem of finding a motion tracker. The firmware runs on the STM32F3DISCOVERY board and should be compatible with all Oculus Rift games.

Development blog
------------------
More details about the reverse engineering can be found on the [development blog](http://yetifrisstlama.blogspot.fr/2014/03/the-foculus-rift-part-2-reverse.html).

Current Status
------------------
I have been testing the headtracker under Ubuntu 13.10 and Windows 7 with various demos intended for the Oculus Rift. It is very usable already and I guess its performance in timing and accuracy is on the same level as the Oculus Rift DK1 headtracker. However, as I don't have an original Rift to compare, I would appreciate your feedback on this.

There is however still an issue with slow drift. In the latest version this has been minimized by a short calibration routine for the gyroscope. Nonetheless, I found out, that the Oculus SDK sporadically disables the correction from the magnetometer for reasons which I do not yet understand. Further work on this might be necesarry.

Things to implement
-----------------------
 * Evaluate the flags, I still do not have a clue what they are used for
 * What is this mysterious 16 bit command field in the Oculus packets for?
 * FixMe: There is this strange synchronization issue, where featureReportData[] sometimes contains invalid data when evaluated in main()


Organization of the code
-------------------------
<pre>
 main.c           main logic of the tracker,
                  handling and reformating the sensor data stream and packing it into 62 byte packets to be sent over USB,
                  Assign the coordinate system directions
                  keeping track of the configuration data structures which the libOVR might send and request
 peripherals.c    setup and request data from the 3 sensor chips over SPI and I2C
                  Handle the scaling and calibration factors, so the headtracker moves in the right way
                  Zero-level calibration routine for the gyroscope
                  Save calibration factors to flash
                  PWM-patterns for the 8 status LEDs
 usb_desc.c       USB - HID descriptors, which fool the PC into thinking that there is an Oculus RIft corrected
 usb_endp.c       STM USB driver endpoint1 callbacks, just sets some global flags to inform the main routine when there is new data
 usb_prop.c       Customization of the STM USB driver, so feature reports can be sent and received
                  received data is copied in the global array featureReportData[] and then processed by the main loop
</pre>

Flashing the STM32F3DISCOVERY board
------------------------------------
*For Ubuntu 13:*

1. Install open-ocd, follow the steps [here](http://engineering-diy.blogspot.fr/2012/11/stm32f3-discovery-eclipse-openocd.html) (only the ones related to open-ocd)

2. Connect the STM32F3DISCOERY board to the PC on its USB ST-LINK connector

3. In a terminal, run this:
```bash
sudo openocd -f /usr/local/share/openocd/scripts/board/stm32f3discovery.cfg -c init -c"reset halt" -c"flash erase_sector 0 0 127" -c"flash write_image stm32f3_HID_for_real.elf"
```

The firmware, which is contained in the stm32f3_HID_for_real.elf binary file, should be flashed. If everything goes well, you can connect the board on the USB USER connector and it should be recognized as: "Oculus VR, Inc. Tracker DK". That's it, mount the board on your HMD and start up the Oculus World Demo.


Calibration and change of orientation
--------------------------------------
To calibrate the Gyroscope for zero-offset (which reduces drift), place the STM board on a flat surface
and push the blue "USER" button for less than 1 second. Make sure that
the board absolutely does not move while the calibration is in progress. The data is permanently stored
in FLASH memory and retained after power down.

To change the reported coordinate system and hence the orientation of the board, push the "USER" button
for > 1 second. Then you will be able to select one out of 8 preconfigured orientation settings,
indicated by the blinking LED. After pushing the "USER" button again for > 1 second, the setting
is also permanently saved to FLASH memory.

ToDo: Add a table here, showing the preconfigured orientations  


Changelog
--------------------------------
<pre>
 09.03.2014:  Fixed bug in handleConfigPacketEvent() by adding break; statements (data rate was always 1 ms before)
 10.03.2014:  Changed wMaxPacketSize from 62 to 64
 19.03.2014:  Changed I2C Bus speed to 400 kHz, which allows to read all 3 sensors in 0.65 ms  (before it was > 2 ms)
 20.03.2014:  Now evaluating the "new data ready" pins of all 3 sensors (improves timing a lot, reduces jitter)
              Enabled FIFO in Streaming mode of Accelerometer and Gyro (no samples will be lost!)
              Fixed Glitches in Magnetometer output by setting it to 75 Hz measurement rate (was 220 Hz before)
 23.03.2014:  Fixed a problem with the USB interrupt and atomic access, not allowing the tracker to change sensor scale
              Changed sensor scaling to floating point numbers and scaled to values as expected from the SDK
 29.03.2014:  Added gyroscope "set to zero" calibration routine (Press the user button on the STM board and keep it very still)
              Added temperature readout from gyro
              Added some nice LED animations for IDLE mode, Tracker running mode and Calibration mode
 01.04.2014:  Gyro offset calibration is now saved to Flash at address 0x08006000 and hence retained after power off
 08.05.2014:  Fixed bug in readSensorAcc(): an array was accessed outside its boundaries.
              also included the .hex file and switched on compiler optimizations
 11.05.2014:  Experimental: Setting of board orientation ...
              Push the user button for > 1 s to choose between 8 preconfigured orientation settings
              Push the user button again for > 1 s to save the setting to FLASH memory

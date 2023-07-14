## __DATALOG1_SensorTile.box Application__

The **FP-SNS-DATALOG1** function pack including **High Speed Datalog** application for **STEVAL-MKSBOX1V1**
provides a comprehensive solution to save data from any combination of sensors and microphones configured up to the maximum sampling rate.
 
The application also allows configuring LSM6DSOX **Machine Learning Core** unit and reading the output of the 
selected algorithm.

Sensor data can be stored onto a micro SD card (Secure Digital High Capacity - SDHC) formatted with the FAT32
file system, or streamed to a PC via USB (WinUSBclass) using the companion host software (cli_example) provided
for Windows and Linux.

The application can be controlled via Bluetooth using the [**STBLESensClassic app**](https://www.st.com/en/embedded-software/stblesensclassic.html)
which lets you manage the board and sensor configurations, start/stop data acquisition on SD card and control 
data labelling.

To read sensor data acquired using FP-SNS-DATALOG1, a few easy-to-use scripts in Python and Matlab are provided
within the software package. The scripts have been successfully tested with MATLAB v2019a and Python 3.10.


### __Keywords__

Datalog, Predictive Maintenance, Condition Monitoring, Signal Processing, Industry 4.0, Sensor, Ultrasound


### __Hardware and Software environment__

  - This example runs on STML4+ Devices.

  - This example has been tested with STMicroelectronics STEVAL-MKSBOX1V1
    evaluation boards and can be easily tailored to any other supported
    device and development board. 


### __How to use it?__

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example


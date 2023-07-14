  ******************************************************************************
  * @file    readme.txt  
  * @author  SRA
  * @brief   This application contains an example which shows how to obtain data
  *          from the various sensors on the STWIN. The data can be saved on SD 
  *          Card
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  ******************************************************************************

How to run PlotSensorData.m or loadDatalogFiles.m

-	Run the script in MATLAB environment.
-	When the application starts, user can select the required sensor from the 
	desired folder. All the data available will be plotted or saved in a struct.
- 	Both the applications are based on the get_subsensorData Matlab class, available
	in the same folder.
- 	The scripts were tested using MATLABv2019a or newer.


How to run ReadSensorDataApp.mlapp

-	Run the application in MATLAB environment.
-	When the application starts, user can select the configuration JSON from the 
	desired folder. 
-	A simple GUI is deployed. User now can select the desired sensor to be plotted.
	User can also plot the spectrogram of the selected sensor.
- 	ReadSensorDataApp.mlapp requires MATLABv2019a or newer.
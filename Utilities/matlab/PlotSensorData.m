% ******************************************************************************
% * @attention
% *
% * Copyright (c) 2022 STMicroelectronics.
% * All rights reserved.
% *
% * This software is licensed under terms that can be found in the LICENSE file
% * in the root directory of this software component.
% * If no LICENSE file comes with this software, it is provided AS-IS.
% *
% *
% ******************************************************************************
%

fclose all;
clear all;
close all;
clc;

[name,fpath] = uigetfile('*.dat','Select one or more STWIN sensors data logs','MultiSelect','on');
if iscell(name)
    fileNames = strings(1, length(name));
    for ii = 1:length(name)
        fileNames(ii) = cell2mat(name(ii));
    end
else
    fileNames = strings(1, 1);
    fileNames(1) = name;
end

configJSONFileName = 'DeviceConfig.json';
acquisitonJSONFileName = 'AcquisitionInfo.json';

cjson = get_jsonFile(configJSONFileName, fpath);
ajson = get_jsonFile(acquisitonJSONFileName, fpath);

for ii = 1:length(fileNames)
    [sensor, subSensorType, units, sensor_idx, sub_idx] = get_subsensorInfo(cjson, fileNames(ii));
    [samples_data, samples_time] = get_subsensorData(cjson, fileNames(ii), fpath);
    
    
    figure
    hold on
    grid on
    
    [n_axes, n_samples] = size(samples_data);
    
    for kk = 1:n_axes
        plot(samples_time,samples_data(kk, :), 'color',rand(1,3))
    end
    ylabel(units)
    xlabel('sec')
    title([sensor, ' - ', subSensorType])


end


function [sensor, subSensor, units, sensor_idx, subSensor_idx] = get_subsensorInfo(cj, fname)
    fileNameCell = strsplit(fname, {'_', '.'});
    sensor = fileNameCell{1};
	subSensor = fileNameCell{2};
    % now find the index
    for se = 1:cj.device.deviceInfo.nSensor
        if strcmp( cj.device.sensor(se).name, sensor )
            sensor_idx = se;
            for su = 1:length( cj.device.sensor(se).sensorDescriptor.subSensorDescriptor )
                if strcmp( cj.device.sensor(se).sensorDescriptor.subSensorDescriptor(su).sensorType, subSensor )
                    subSensor_idx = su;
                    units = cj.device.sensor(se).sensorDescriptor.subSensorDescriptor(su).unit;
                end
            end
        end
    end
end


function fjson = get_jsonFile(fileJSON, fpath)
    fullName = strcat(fpath, fileJSON);
    fid = fopen(fullName);
    str = fread(fid,inf);
    fclose(fid);
    fjson = jsondecode(char(str)');
end

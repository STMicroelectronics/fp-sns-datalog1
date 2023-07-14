function [log, s_time] = get_subsensorData(cj, filename, path)
    
        fileNameCell = strsplit(filename, {'_', '.'});
        sensor = fileNameCell{1};
        subSensor = fileNameCell{2};
        % now find the index
        for se = 1:cj.device.deviceInfo.nSensor
            if strcmp( cj.device.sensor(se).name, sensor )
                sensor_idx = se;
                for su = 1:length( cj.device.sensor(se).sensorDescriptor.subSensorDescriptor )
                    if strcmp( cj.device.sensor(se).sensorDescriptor.subSensorDescriptor(su).sensorType, subSensor )
                        subSensor_idx = su;
                    end
                end
            end
        end
        
        fileDAT = strcat(path, filename);
        fid = fopen(fileDAT);
        rawdata = fread(fid,inf,'uint8=>uint8');
        fclose(fid);
        
        [log, s_time] = decodeRawdata(cj.device.sensor(sensor_idx), subSensor_idx, rawdata);
        
end


function [sData, samples_time] = decodeRawdata(cj, sub_idx, raw_data)

    subSensorType = cj.sensorDescriptor.subSensorDescriptor(sub_idx).sensorType;
    n_axes = cj.sensorDescriptor.subSensorDescriptor(sub_idx).dimensions;
    dataType_size = checkTypeLength(cj.sensorDescriptor.subSensorDescriptor(sub_idx).dataType);
    dataType = checkType(cj.sensorDescriptor.subSensorDescriptor(sub_idx).dataType);
    
    samplesPerTs = cj.sensorStatus.subSensorStatus(sub_idx).samplesPerTs;
    if samplesPerTs == 0
        timeStamp_size = 0;
        dataFrame_size = length(raw_data);        
    else
        timeStamp_size = checkTypeLength('double');
        dataFrame_size = samplesPerTs*sum(n_axes)*dataType_size;       
    end

    frame_size = dataFrame_size + timeStamp_size;
    
    if strcmp(subSensorType, 'MLC')
        framePeriod = 0;
        checkTimeStamps = 0;
    else
        framePeriod = samplesPerTs/cj.sensorStatus.subSensorStatus(sub_idx).ODR;
        checkTimeStamps = 1;
    end

    num_frames = floor(length(raw_data)/frame_size);

    if num_frames == 0
        %warning('No valid data avilable for %s in sensor %s ', cj.sensorDescriptor.subSensorDescriptor(su).sensorType, cj.name);
    else
        rndDataBuffer = raw_data(1:(frame_size * num_frames));
        samplesPerFrame = dataFrame_size / dataType_size;
        if samplesPerTs == 0
            samplesPerTs = samplesPerFrame;
        end
        
        data = zeros([num_frames*samplesPerFrame, 1],dataType);
        timeStamp = zeros([num_frames, 1],checkType('double'));

        for jj = 1:num_frames
            startFrame = (jj-1)*frame_size+1;
            segmentTS = rndDataBuffer(startFrame+dataFrame_size:startFrame+frame_size-1);
            segmentData = rndDataBuffer(startFrame:startFrame+dataFrame_size-1);
            if ~isempty(segmentTS)
                timeStamp(jj) = typecast(segmentTS, checkType('double'));
            end
            data( (jj-1)*samplesPerFrame+1:(jj*samplesPerFrame) ) = typecast(segmentData, dataType);

            % Check Timestamp consistency
            if (checkTimeStamps == 1 && jj > 1)
                deltaTS = timeStamp(jj)-timeStamp(jj-1);
                if abs(deltaTS)<0.66*framePeriod || abs(deltaTS)>1.33*framePeriod || isnan(timeStamp(jj))|| isnan(timeStamp(jj-1))
                    data( (jj-1)*samplesPerFrame+1:(jj*samplesPerFrame) ) = 0;
                    %warning('WARNING Sensor %s: corrupted data at %s sec', cj.name, timeStamp(jj));
                    timeStamp(jj) = timeStamp(jj-1)+framePeriod;
                end
            end
        end

        timestamp_first = cj.sensorStatus.subSensorStatus(sub_idx).initialOffset;
        timeStamp = [timestamp_first; timeStamp];
        
        if num_frames == 1
            timeStamp = [timeStamp; framePeriod];
        end

        sensitivity = cj.sensorStatus.subSensorStatus(sub_idx).sensitivity;
        sData = reshape(data, [n_axes, num_frames*samplesPerTs]);
        sData = cast(sData,'single') .* sensitivity;
        
        samples_time = zeros([1, num_frames*samplesPerTs]);
        for jj = 1:num_frames
            a = linspace(timeStamp(jj),timeStamp(jj+1),samplesPerTs+1);
            samples_time(((jj-1)*samplesPerTs+1):(jj*samplesPerTs)) = a(2:end);
        end
        
    end

end

function type = checkType(type)

switch type
    case 'uint8_t'
        type = 'uint8';
    case 'uint16_t'
        type = 'uint16';
    case 'uint32_t'
        type = 'uint32';
    case 'int8_t'
        type = 'int8';
    case 'int16_t'
        type = 'int16';
    case 'int32_t'
        type = 'int32';
    case 'float'
        type = 'single';
    case 'double'
        type = 'double';
end

end

function typeLength = checkTypeLength(type)

switch type
    case {'uint8_t','int8_t'}
        typeLength = 1;
    case {'uint16_t','int16_t'}
        typeLength = 2;
    case {'uint32_t','int32_t'}
        typeLength = 4;
    case 'float'
        typeLength = 4;
    case 'double'
        typeLength = 8;
end

end

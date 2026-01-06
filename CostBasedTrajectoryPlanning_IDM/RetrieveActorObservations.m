function [AllActorObservationState] = RetrieveActorObservations...
    (scenario, sensors)

    time = scenario.SimulationTime;
    numSensors = numel(sensors);

    AllActorObservationState = struct( ...
        'SensorIndex', {}, ...
        'SensorType', {}, ...
        'MeasuredQuantity', {}, ...
        'RawData', {} );

    % % scenario はこの関数では進めない
    % isRunning = true;

    for i = 1:numSensors
        sensor = sensors{i};

        % --- センサタイプ判定 ---
        if isa(sensor, 'drivingRadarDataGenerator')
            sensorType = "Radar";
            measuredQuantity = "Relative range, relative velocity, azimuth angle";

            poses = targetPoses(scenario, sensor.SensorIndex);
            [detections, isValid] = sensor(poses, time);

        elseif isa(sensor, 'visionDetectionGenerator')
            sensorType = "Camera";
            measuredQuantity = "Object class and image-plane position";

            poses = targetPoses(scenario, sensor.SensorIndex);
            [detections, isValid] = sensor(poses, time);

        elseif isa(sensor, 'insSensor')
            sensorType = "INS";
            measuredQuantity = "Ego position, velocity, angular rate";

            [detections, isValid] = sensor(time);

        else
            sensorType = "Unknown";
            measuredQuantity = "Unknown";
            detections = [];
            isValid = false;
        end

        % --- 有効な観測のみ保存 ---
        if isValid
            AllActorObservationState(end+1).SensorIndex      = sensor.SensorIndex; %#ok<SAGROW>
            AllActorObservationState(end).SensorType         = sensorType;
            AllActorObservationState(end).MeasuredQuantity   = measuredQuantity;
            AllActorObservationState(end).RawData            = detections;
        end
    end
end

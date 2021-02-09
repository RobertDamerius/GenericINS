%% INITIALIZATION
    % Add source path and load example data
    clear all;
    addpath(['..' filesep '..' filesep 'source']);
    load(['..' filesep 'SensorData.mat']);
    input.imu.timestamp  = sensor.imu.timestamp;
    input.imu.data       = [sensor.imu.accelerationX sensor.imu.accelerationY sensor.imu.accelerationZ sensor.imu.angularRateX sensor.imu.angularRateY sensor.imu.angularRateZ];
    input.gnss.timestamp = sensor.gnss.timestamp;
    input.gnss.data      = [sensor.gnss.latitude sensor.gnss.longitude sensor.gnss.altitude];
    input.dvl.timestamp  = sensor.dvl.timestamp;
    input.dvl.data       = [sensor.dvl.velocityX sensor.dvl.velocityY sensor.dvl.velocityZ];
    input.ahrs.timestamp = sensor.ahrs.timestamp;
    input.ahrs.data      = [sensor.ahrs.roll sensor.ahrs.pitch sensor.ahrs.yaw];

    % Generate generic INS object.
    ins = GenericINS();

    % We need to specify sensor configurations, that is sensor alignment and sensor characteristics, let's use a struct to store those parameters
    config.imu.positionBody2Sensor  = [1.032;-0.5;1.564];
    config.imu.dcmBody2Sensor       = eye(3);
    config.imu.stdAcc               = 0.0014142135623731 * ones(3,1);
    config.imu.stdGyr               = 1.4142135623731e-05 * ones(3,1);
    config.imu.stdAccBias           = 3.16227766016838e-13 * ones(3,1);
    config.imu.stdGyrBias           = 3.16227766016838e-13 * ones(3,1);
    config.gnss.positionBody2Sensor = [1.49; 0.18; 0.06];
    config.gnss.stdNED              = [0.01; 0.01; 0.1];
    config.dvl.positionBody2Sensor  = [1.09; 0.88; 1.67];
    config.dvl.dcmBody2Sensor       = eye(3);
    config.dvl.stdXYZ               = 3e-3 * ones(3,1);
    config.ahrs.dcmBody2Sensor      = eye(3);
    config.ahrs.stdRollPitchYaw     = 1e-4 * ones(3,1);

    % Initialize the INS
    % Actually the initial state must be given for the position of the IMU but for simplicity we use the first sensor measurements as initial state and a high
    % uncertainty. We don't know the initial inertial biases so we assume zero.
    initialPositionLLA              = input.gnss.data(1,:)';
    initialVelocityUVW              = input.dvl.data(1,:)';
    initialOrientationRollPitchYaw  = input.ahrs.data(1,:)';
    initialBiasAcc                  = zeros(3,1);
    initialBiasGyr                  = zeros(3,1);
    initialStdPositionLLA           = [1e-8; 1e-8; 0.1];
    initialStdVelocityUVW           = [0.1; 0.1; 0.1];
    initialStdOrientationVector     = [1e-3; 1e-3; 1e-3];
    initialStdBiasAcc               = [1e-4; 1e-4; 1e-4];
    initialStdBiasGyr               = [1e-4; 1e-4; 1e-4];
    fprintf('Initializing INS: ');
    ins.Initialize(initialPositionLLA, initialVelocityUVW, initialOrientationRollPitchYaw, initialBiasAcc, initialBiasGyr, initialStdPositionLLA, initialStdVelocityUVW, initialStdOrientationVector, initialStdBiasAcc, initialStdBiasGyr, config.imu.stdAcc, config.imu.stdGyr, config.imu.stdAccBias, config.imu.stdGyrBias);
    fprintf('OK\n');

%% RUN THE GENERIC INS ALGORITHM
    buffer.imu.trigger    = false;
    buffer.imu.timestamp  = input.imu.timestamp(1);
    buffer.imu.data       = input.imu.data(1,:)';
    buffer.gnss.trigger   = false;
    buffer.gnss.timestamp = input.gnss.timestamp(1);
    buffer.gnss.data      = input.gnss.data(1,:)';
    buffer.dvl.trigger    = false;
    buffer.dvl.timestamp  = input.dvl.timestamp(1);
    buffer.dvl.data       = input.dvl.data(1,:)';
    buffer.ahrs.trigger   = false;
    buffer.ahrs.timestamp = input.ahrs.timestamp(1);
    buffer.ahrs.data      = input.ahrs.data(1,:)';
    numSamples = size(sensor.imu.timestamp,1);
    output.latitude = nan(numSamples,1);
    output.longitude = nan(numSamples,1);
    fprintf('Running INS:      000 %%');
    for k=2:numSamples
        percent = int32(floor(100*k/numSamples));
        fprintf('\b\b\b\b\b%03d %%',percent);

        % Elapsed time to previous IMU measurement
        sampletime = NaN;

        % Convert sensor timestamps to measurement triggers: we need to now if a new measurement occurred
        if(input.imu.timestamp(k) > buffer.imu.timestamp)
            sampletime = input.imu.timestamp(k) - buffer.imu.timestamp;
            buffer.imu.trigger = true;
            buffer.imu.timestamp = input.imu.timestamp(k);
            buffer.imu.data = input.imu.data(k,:)';
        end
        if(input.gnss.timestamp(k) > buffer.gnss.timestamp)
            buffer.gnss.trigger = true;
            buffer.gnss.timestamp = input.gnss.timestamp(k);
            buffer.gnss.data = input.gnss.data(k,:)';
        end
        if(input.dvl.timestamp(k) > buffer.dvl.timestamp)
            buffer.dvl.trigger = true;
            buffer.dvl.timestamp = input.dvl.timestamp(k);
            buffer.dvl.data = input.dvl.data(k,:)';
        end
        if(input.ahrs.timestamp(k) > buffer.ahrs.timestamp)
            buffer.ahrs.trigger = true;
            buffer.ahrs.timestamp = input.ahrs.timestamp(k);
            buffer.ahrs.data = input.ahrs.data(k,:)';
        end

        % Actual INS algorithm
        if(buffer.imu.trigger)
            buffer.imu.trigger = false;

            % Predict motion with IMU
            ins.Predict(buffer.imu.data, sampletime, config.imu.dcmBody2Sensor, config.imu.positionBody2Sensor);

            % Update with new sensor data
            if(buffer.gnss.trigger)
                buffer.gnss.trigger = false;
                ins.UpdatePosition3D(buffer.gnss.data, config.gnss.stdNED, config.gnss.positionBody2Sensor);
            end
            if(buffer.dvl.trigger)
                buffer.dvl.trigger = false;
                ins.UpdateVelocity3D(buffer.dvl.data, config.dvl.stdXYZ, config.dvl.positionBody2Sensor, config.dvl.dcmBody2Sensor);
            end
            if(buffer.ahrs.trigger)
                buffer.ahrs.trigger = false;
                ins.UpdateOrientation3D(buffer.ahrs.data, config.ahrs.stdRollPitchYaw, config.ahrs.dcmBody2Sensor);
            end
        end

        % Get final state from INS
        [valid, positionLLA, orientationQuaternionWXYZ, orientationRollPitchYaw, velocityNED, velocityUVW, velocityPQR, courseOverGround, speedOverGround, angleOfAttack, sideSlipAngle, inertialSensorBias] = ins.GetMotionState();
        output.latitude(k) = positionLLA(1);
        output.longitude(k) = positionLLA(2);
    end
    fprintf('\n');

%% PLOT RESULTS
    fprintf('Plotting results: ');
    % Convert geographic position (lat, lon) to tangential plane (x, y) for small distances
    origin = mean([reference.latitude reference.longitude reference.altitude])';
    s = sin(origin(1));
    invRoot = 1.0 / sqrt(1 - 0.0066943799901410724656084644 * s * s);
    R0 = sqrt(40408299984661.5 * invRoot^4);
    positionRef = zeros(size(reference.latitude,1),2);
    for k = 1:size(positionRef,1)
        positionRef(k,1) = (R0 + origin(3)) * (reference.latitude(k) - origin(1));
        positionRef(k,2) = (R0 + origin(3)) * (reference.longitude(k) - origin(2))*cos(0.5 * (origin(1) + reference.latitude(k)));
    end
    positionEst = zeros(size(output.latitude,1),2);
    for k = 1:size(positionEst,1)
        positionEst(k,1) = (R0 + origin(3)) * (output.latitude(k) - origin(1));
        positionEst(k,2) = (R0 + origin(3)) * (output.longitude(k) - origin(2))*cos(0.5 * (origin(1) + output.latitude(k)));
    end

    % Plot position comparison
    figure(1); clf; hold on;
    plot(positionRef(:,1), positionRef(:,2), 'k--');
    plot(positionEst(:,1), positionEst(:,2));
    legend('reference','estimate');
    title('Trajectory');
    xlabel('North (m)');
    ylabel('East (m)');
    view(90,-90); grid on; box on; axis equal;
    fprintf('OK\n');

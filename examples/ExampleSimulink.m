%% INITIALIZATION
    % Add source path
    addpath(['..' filesep 'source']);

    % Load data
    load SensorData.mat;
    input = [sensor.imu.timestamp sensor.imu.accelerationX sensor.imu.accelerationY sensor.imu.accelerationZ sensor.imu.angularRateX sensor.imu.angularRateY sensor.imu.angularRateZ, ...
             sensor.gnss.timestamp sensor.gnss.latitude sensor.gnss.longitude sensor.gnss.altitude, ...
             sensor.dvl.timestamp sensor.dvl.velocityX sensor.dvl.velocityY sensor.dvl.velocityZ, ...
             sensor.ahrs.timestamp sensor.ahrs.roll sensor.ahrs.pitch sensor.ahrs.yaw];

    % Open and initialize model
    modelName = 'ExampleSimulinkModel';
    sampletime = 0.05;
    simulationTime = (0:sampletime:sampletime*(size(input,1)-1))';
    open([modelName '.slx']);
    set_param(modelName,'StopTime',num2str(sampletime*(size(input,1)-1)),'FixedStep',num2str(sampletime));

    % Set input
    simIn = Simulink.SimulationInput(modelName);
    simIn.ExternalInput = timeseries(input, simulationTime);


%% SIMULATE AND GET OUTPUT
    simOut = sim(simIn);
    output = squeeze(simOut.yout{1}.Values.Data);
    estimate = struct('latitude',output(:,1),'longitude',output(:,2));


%% PLOT RESULTS
    % Convert geodetic position (lat, lon) to tangential plane (x, y) for small distances
    origin = mean([reference.latitude reference.longitude reference.altitude])';
    s = sin(origin(1));
    invRoot = 1.0 / sqrt(1 - 0.0066943799901410724656084644 * s * s);
    R0 = sqrt(40408299984661.5 * invRoot^4);
    positionRef = zeros(size(reference.latitude,1),2);
    for k = 1:size(positionRef,1)
        positionRef(k,1) = (R0 + origin(3)) * (reference.latitude(k) - origin(1));
        positionRef(k,2) = (R0 + origin(3)) * (reference.longitude(k) - origin(2))*cos(0.5 * (origin(1) + reference.latitude(k)));
    end
    positionEst = zeros(size(estimate.latitude,1),2);
    for k = 1:size(positionEst,1)
        positionEst(k,1) = (R0 + origin(3)) * (estimate.latitude(k) - origin(1));
        positionEst(k,2) = (R0 + origin(3)) * (estimate.longitude(k) - origin(2))*cos(0.5 * (origin(1) + estimate.latitude(k)));
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

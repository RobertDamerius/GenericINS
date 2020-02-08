# Overview
The Generic Inertial Navigation System (INS) uses a square-root spherical simplex unscented Kalman filter (SRSSUKF) to estimate the navigation state.
`SRSSUKF.m` is a MATLAB class implementation of the SRSSUKF that can be used for any state estimation problem.
The actual Generic INS is implemented in the MATLAB class `GenericINS.m`. This class reimplements the SRSSUKF algorithm for the Generic INS problem,
that is, the SRSSUKF MATLAB class is not required but it is left in the source directory.

The generic estimate the state vector containing:
- geographic position (latitude, longitude, altitude)
- velocity in north, east and down direction
- orientation (unit quaternion)
- inertial sensor bias (accelerometer and gyroscope)

The state is estimated for the position of the IMU (accelerometer and gyroscope) and can be transformed to any point of interest.

**Use of the GenericINS class**<br>
You can use the GenericINS class in MATLAB or Simulink (with embedded MATLAB functions). First, you need to create a GenericINS object.
```
genericINS = GenericINS();
```
Then you need to initialize the INS with an initial state and initial uncertainty.
```
genericINS.Initialize(PARAMETERS);
```
To get more information about the parameters required to initialize the filter, simply type `help GenericINS.Initialize`. You can
also call this member function if you want to reset the INS. Once the INS is initialized you can make predictions using inertial measurement data
and update the state estimation with additional sensor data using one of the update methods. You can update position, velocity and orientation
in either 1, 2 or 3 dimensions. Note that orientation updates should only be used for applications with small roll and pitch angles.
The INS uses a unit quaternion for the representation of the orientation, that is, no singularities can occur. However, updates with euler angles
can cause problems because they are not handled correctly during measurement updates. Imagine you want to support the INS with a GNSS (3D position)
and a 2-dimensional velocity sensor. Then the main structure would be as follows.
```
for k=1:kmax
    genericINS.Predict(imuData, sampletime, dcmIMUBody2Sensor, posIMUBody2Sensor);

    genericINS.UpdatePosition3D(measurementLatLonAlt, stdNorthEastDown, posBody2Sensor);
    genericINS.UpdateVelocity2D(measurementXY, stdXY, posBody2Sensor, dcmBody2Sensor);
end
```
Take a look to the corresponding help pages for more information about the parameters.

## Automatic function generation
The GenericINS MATLAB class can automatically generate a function to be used as an embedded MATLAB function in Simulink.
Imagine, you have 2 GNSS-receivers (both providing latitude, longitude, altitude), 3 one-dimensional velocity sensors and one compass. The MATLAB function would be generated as follows.
```
functionName = 'MyINS';           % Give the function a name
positionMeasurements = [3 3];     % Dimensions {1, 2 or 3} for all position measurements
velocityMeasurements = [1 1 1];   % Dimensions {1, 2 or 3} for all velocity measurements
orientationMeasurements = [1];    % Dimensions {1, 2 or 3} for all orientation measurements

% Create "MyINS.m"
GenericINS.GenerateFunction(functionName, positionMeasurements, velocityMeasurements, orientationMeasurements);
```
The generated function includes a documentation (comment) that helps you to understand all input and output arguments.

Note that an IMU is essential as these intertial measurements are required to predict the movement of the vehicle.
Additional sensor measurements can only be processed in sequential updates if a state prediction has been made beforehand.
The generated function uses an internal buffer to store the latest sensor data.
A prediction step is only performed when a new IMU measurement has been received.
Update steps for additional sensors are only performed if a prediction step has been performed and the measurement of the corresponding sensor marks a new, previously unused measurement.
Therefore, the IMU measurement rate should be as high as possible.

Take a look to [examples](../examples/) for MATLAB/Simulink examples that use real sensor data.


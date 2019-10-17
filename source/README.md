# Overview
The Generic Inertial Navigation System (INS) uses a square-root spherical simplex unscented Kalman filter (SRSSUKF) to estimate the navigation state.
`SRSSUKF.m` is a MATLAB class implementation of the SRSSUKF that can be used for any state estimation problem.
The actual Generic INS is implemented in the MATLAB class `GenericINS.m`. It uses the SRSSUKF to estimate the state vector containing:
- geodetic position (latitude, longitude, altitude)
- velocity in north, east and down direction
- orientation (unit quaternion)
- inertial sensor bias (accelerometer only)

The state is estimated for the position of the accelerometer and can be transformed to any point of interest.
All parameters including sensor alignment and filter tuning are defined in the MATLAB class `GenericINSParameters.m`.

**Current implementation**<br>
In the current implementation, the following sensors are used:
- Inertial Measurement Unit (IMU)
- Doppler Velocity Log (DVL)
- Global Navigation Satellite System (GNSS)-Receiver
- Attitude-Heading Reference System (AHRS)

A strapdown algorithm is used to predict the state based on acceleration and angular rate measurements from the IMU.
The earth's rotation is taken into account.

## Using the GenericINS
If your sensor configuration is similar to (or only a subset of) the above, you can use the GenericINS directly.
Note that an IMU is essential as these intertial measurements are required to predict the movement of the vehicle.
Additional sensor measurements can only be processed in sequential updates if a state prediction has been made beforehand.

In MATLAB, create a GenericINS object.
```
geneicINS = GenericINS();
```
If new measurements are available, call the step member function.
```
imuData = [timestamp; accX; accY; accZ; gyrX; gyrY; gyrZ];
gnssData = [timestamp; lat; lon; alt];
dvlData = [timestamp; velX; velY; velZ];
ahrsData = [timestamp; roll; pitch; yaw];

genericINS.Step(imuData, gnssData, dvlData, ahrsData);
```
Take a look to [examples](../examples/) for a MATLAB/Simulink example that uses real sensor data.

Note that each sensor data vector contains a timestamp at the beginning.
The timestamp is used to detect whether a new measurement has been received or not.
The GenericINS uses an internal buffer to store the latest sensor data.
A prediction step is only performed when a new IMU measurement has been received.
Update steps for additional sensors are only performed if a prediction step has been performed and the measurement of the corresponding sensor marks a new, previously unused measurement.
Therefore, the IMU measurement rate should be as high as possible.

**Excluding sensors**<br>
If the corresponding sensor is not available or the measurement is to be rejected, any element of the sensor data vector can be set to NaN.


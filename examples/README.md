# Examples
As an example real sensor data (more than 3 hours) from trials with an unmanned surface vehicle (USV) are used.
The vehicle is equipped with the following sensors:
- Inertial Measurement Unit (IMU)
- Doppler Velocity Log (DVL)
- Global Navigation Satellite System (GNSS)-Receiver
- Attitude-Heading Reference System (AHRS)

There are two ways to run the example - either MATLAB or Simulink.
Make sure your MATLAB/Simulink version is at least R2020b.
The MATLAB example may also work with previous versions.
However, it is recommended to run the Simulink example as this is much faster than the MATLAB example.

**MATLAB Example**<br>
Simply run the `ExampleMATLAB.m` script in the MATLAB directory.

**Simulink Example**<br>
Simply run the `ExampleSimulink.m` script in the Simulink directory.
This example uses the Simulink library which contains a sensor fusion algorithm for up to 20 sensors.
For most applications, the number of different sensors used by this algorithm is sufficient.
If this number of predefined sensors does not fit your application, you can generate an embeded MATLAB function yourself.
In this case take a look to the SensorFusion MATLAB class in the [package](../packages/+GenericINS/) directory.


% GenericINSParameters.m
%
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Version     Author                 Changes
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 20181009    Robert Damerius        Initial release.
% 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 
% 
%GenericINSParameters
% This class provides constant parameters for the generic inertial navigation system, that is installation locations and orientations
% for all sensors and tuning parameters (variance or standard deviation).

classdef GenericINSParameters
    properties(Constant = true)
        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        % SENSOR ALIGNMENT
        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        % A high-precision differential GNSS is used as reference. The reference GNSS location is used as origin for the body
        % frame. All sensor positions are therefore given with respect to the reference GNSS. This makes it easy to compare the
        % sensor fusion result (position) with the reference position.
        ACC_POSITION_B2S     = [1.032;-0.5;1.564];     % [m] Position of accelerometer w.r.t. body origin.
        ACC_ORIENTATION_S2B  = eye(3);                 % Rotation matrix to transform acceleration from sensor to body frame.
        GYR_ORIENTATION_S2B  = eye(3);                 % Rotation matrix to transform angular rate from sensor to body frame.
        GNSS_POSITION_B2S    = [1.49; 0.18; 0.06];     % [m] Position of GNSS-antenna w.r.t. body origin.
        DVL_POSITION_B2S     = [1.09; 0.88; 1.67];     % [m] Position of DVL w.r.t. body origin.
        DVL_ORIENTATION_B2S  = eye(3);                 % Rotation matrix to transform velocity from body to sensor frame.


        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        % SENSOR CHARACTERISTICS
        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        % For filter tuning the covariance matrices Q and R are required. They are calculated from sensor characteristics.
        % The following values are empirically adjusted and in no way optimized.
        ACC_VARIANCE_X                = 2.0e-6;        % [(m/s^2)^2] Variance of accelerometer measurements in x direction.
        ACC_VARIANCE_Y                = 2.0e-6;        % [(m/s^2)^2] Variance of accelerometer measurements in y direction.
        ACC_VARIANCE_Z                = 2.0e-6;        % [(m/s^2)^2] Variance of accelerometer measurements in z direction.
        GYR_VARIANCE_X                = 2.0e-10;       % [(rad/s)^2] Variance of gyroscope measurements in x direction.
        GYR_VARIANCE_Y                = 2.0e-10;       % [(rad/s)^2] Variance of gyroscope measurements in y direction.
        GYR_VARIANCE_Z                = 2.0e-10;       % [(rad/s)^2] Variance of gyroscope measurements in z direction.
        ACC_BIAS_VARIANCE_X           = 1.0e-25;       % [(m/s^2)^2] Variance of accelerometer bias in x direction.
        ACC_BIAS_VARIANCE_Y           = 1.0e-25;       % [(m/s^2)^2] Variance of accelerometer bias in y direction.
        ACC_BIAS_VARIANCE_Z           = 1.0e-25;       % [(m/s^2)^2] Variance of accelerometer bias in z direction.
        GNSS_STANDARD_DEVIATION_NORTH = 0.004;         % [m] GNSS position standard deviation in north direction.
        GNSS_STANDARD_DEVIATION_EAST  = 0.004;         % [m] GNSS position standard deviation in east direction.
        GNSS_STANDARD_DEVIATION_DOWN  = 0.008;         % [m] GNSS position standard deviation in down direction.
        DVL_STANDARD_DEVIATION_X      = 3e-3;          % [m/s] DVL velocity standard deviation in x direction.
        DVL_STANDARD_DEVIATION_Y      = 3e-3;          % [m/s] DVL velocity standard deviation in y direction.
        DVL_STANDARD_DEVIATION_Z      = 3e-3;          % [m/s] DVL velocity standard deviation in z direction.
        AHRS_STANDARD_DEVIATION_ROLL  = 1e-4;          % [rad] Roll angle standard deviation.
        AHRS_STANDARD_DEVIATION_PITCH = 1e-4;          % [rad] Pitch angle standard deviation.
        AHRS_STANDARD_DEVIATION_YAW   = 1e-4;          % [rad] Yaw angle standard deviation.


        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        % GENERAL FILTER SETTINGS
        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        WEIGHT_0TH_SIGMAPOINT = 0.1                        % Weight for 0-th sigma point, range: (0; 1).

        % Diagonal of the initial covariance matrix (l l a v v v ov ov ov ba ba ba)
        DIAG_P0 = [1e-16; 1e-16; 1e-2; 1e-2; 1e-2; 1e-2; 1e-7; 1e-7; 1e-7; 2e-9; 2e-9; 2e-9];

        % Initial inertial biases
        ACC_INITIAL_BIAS = [-6.0e-4; -2.0e-3; -1.5e-3];   % [m/s^2] Initial bias for accelerometer.
    end
end


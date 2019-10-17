% GenericINS.m
%
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Version     Author                 Changes
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 20181009    Robert Damerius        Initial release.
% 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 
% 
%GenericINS
% A generic inertial navigation system that fuses sensor data from IMU (inertial measurement unit), DVL (doppler velocity log),
% GNSS-Receiver (global navigation satellite system) and AHRS (attitude-heading reference system). The process model and the
% sensor models are given as static member functions. They are used as callback functions for the internal square-root spherical
% simplex unscented kalman filter (SRSSUKF). You just need to call the Step() member function and provide all sensor data.
% Using the timestamp of the sensor data the GenericINS automatically detects which sensor data to use to update the state
% estimation. Use the GetState() member function to get the estimated state according to the body-frame. You can use the Reset()
% member function to reset the filter.
% Parameters like sensor alignment and sensor characteristics are stored in a separate class (GenericINSParameters). The
% GenericINS class reads the constant parameters from the GenericINSParameters class.

classdef GenericINS < handle
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % PUBLIC MEMBER FUNCTIONS
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    methods
        function obj = GenericINS()
            %GenericINS.GenericINS Create a GenericINS object.

            % State dimension (l l a v v v q q q q b b b)
            xDim = int32(13);

            % Dimension for process noise (a a a g g g b b b)
            wDim = int32(9);

            % Specify angle and quaternion indices for state vector (filter must not operate in high latitudes)
            xIdxAngle = int32(2);
            xIdxQuaternion = int32(7);

            % Dimension for sensor models: GNSS(3) DVL(3) AHRS(3)
            cyDim = {int32(3), int32(3), int32(3)};

            % No state augmentation for sensor uncertainty, noise is assumed to be additive
            cvDim = {int32(0), int32(0), int32(0)};

            % Create actual filter object
            obj.srssukf = SRSSUKF(GenericINSParameters.WEIGHT_0TH_SIGMAPOINT, xDim, wDim, xIdxAngle, xIdxQuaternion, cyDim, cvDim);

            % Initialize properties
            [obj.omegaEarth, obj.Rn, obj.Re, obj.R0] = GenericINS.WGS84(0.0);
            obj.localGravity = [0.0; 0.0; GenericINS.Gravity(0.0, 0.0, obj.R0)];
            obj.initialized = false;
            obj.imuBuffer = nan(7,1);
            obj.gnssBuffer = nan(4,1);
            obj.dvlBuffer = nan(4,1);
            obj.ahrsBuffer = nan(4,1);
            obj.gnssTrigger = false;
            obj.dvlTrigger = false;
            obj.ahrsTrigger = false;
        end
        function Step(obj, imuData, gnssData, dvlData, ahrsData)
            %GenericINS.Step The discrete-time step function of the sensor fusion system. Use this function to provide new sensor measurements to the multi-sensor filter and perform an estimation step.
            % 
            % PARAMETERS
            % imuData  ... 7x1 vector: [timestamp (s); accX (m/s^2); accY (m/s^2); accZ (m/s^2); gyrX (rad/s); gyrY (rad/s); gyrZ (rad/s)].
            % gnssData ... 4x1 vector: [timestamp (s); lat (rad); lon (rad); alt (m, positive upwards)].
            % dvlData  ... 4x1 vector: [timestamp (s); velX (m/s); velY (m/s); velZ (m/s)].
            % ahrsData ... 4x1 vector: [timestamp (s); roll (rad); pitch (rad); yaw (rad)].

            % Update sensor buffers
            dt = 0;
            if((~logical(sum(isnan(imuData)))) && ((imuData(1) > obj.imuBuffer(1)) || logical(sum(isnan(obj.imuBuffer)))))
                if(~logical(sum(isnan(obj.imuBuffer))))
                    dt = imuData(1) - obj.imuBuffer(1);
                end
                obj.imuBuffer = imuData;
            end
            if((~logical(sum(isnan(gnssData)))) && ((gnssData(1) > obj.gnssBuffer(1)) || logical(sum(isnan(obj.gnssBuffer)))))
                obj.gnssBuffer = gnssData;
                obj.gnssTrigger = true;
            end
            if((~logical(sum(isnan(dvlData)))) && ((dvlData(1) > obj.dvlBuffer(1)) || logical(sum(isnan(obj.dvlBuffer)))))
                obj.dvlBuffer = dvlData;
                obj.dvlTrigger = true;
            end
            if((~logical(sum(isnan(ahrsData)))) && ((ahrsData(1) > obj.ahrsBuffer(1)) || logical(sum(isnan(obj.ahrsBuffer)))))
                obj.ahrsBuffer = ahrsData;
                obj.ahrsTrigger = true;
            end

            % Initialization routine
            if(~obj.initialized)
                if(obj.gnssTrigger)
                    p0 = obj.gnssBuffer(2:4);
                    v0 = zeros(3,1);
                    q0 = [1.0; 0.0; 0.0; 0.0];
                    ba0 = GenericINSParameters.ACC_INITIAL_BIAS;
                    obj.Initialize(p0, v0, q0, ba0);
                    obj.gnssTrigger = false;
                    obj.dvlTrigger = false;
                    obj.ahrsTrigger = false;
                    obj.initialized = true;
                end
                return;
            end

            % Run estimation (Predict/Update) if new IMU data available
            if(dt > 0.0)
                % Predict state
                obj.Predict(obj.imuBuffer(2:7), dt);

                % If sensor data available, perform sequential updates
                if(obj.gnssTrigger)
                    obj.gnssTrigger = false;
                    obj.UpdateGNSS(obj.gnssBuffer(2:4));
                end
                if(obj.dvlTrigger)
                    obj.dvlTrigger = false;
                    obj.UpdateDVL(obj.dvlBuffer(2:4), obj.imuBuffer(5:7));
                end
                if(obj.ahrsTrigger)
                    obj.ahrsTrigger = false;
                    obj.UpdateAHRS(obj.ahrsBuffer(2:4));
                end
            end
        end
        function [positionLLA, velocityNED, velocityUVW, velocityPQR, quaternion, roll, pitch, yaw] = GetState(obj)
            %GenericINS.GetState Return the current state estimation transformed back to the point of interest.
            % 
            % RETURN
            % positionLLA     ... Position according to WGS84: lat (rad), lon (rad), altitude (m, positive upwards).
            % velocityNED     ... Translational velocity in the navigation frame: vNorth (m/s), vEast (m/s), vDown (m/s).
            % velocityUVW     ... Translational velocity in the body frame: u (m/s), v (m/s), w (m/s).
            % velocityPQR     ... Rotational velocity in the body frame: p (rad/s), q (rad/s), r (rad/s).
            % quaternion      ... Unit quaternion representing a rotation from the body frame to the navigation frame.
            % roll            ... Roll angle in radians (euler angle, ZYX-convention).
            % pitch           ... Pitch angle in radians (euler angle, ZYX-convention).
            % yaw             ... Yaw angle in radians (euler angle, ZYX-convention).

            % Set default values (NaN)
            positionLLA = nan(3,1);
            velocityNED = nan(3,1);
            velocityUVW = nan(3,1);
            velocityPQR = nan(3,1);
            quaternion = nan(4,1);
            roll = NaN;
            pitch = NaN;
            yaw = NaN;

            % Get estimates if filter is initialized
            if(obj.initialized)
                % Get state estimation from SRSSUKF (w.r.t. accelerometer position)
                x = obj.srssukf.GetState();

                % Set orientation
                quaternion = GenericINS.Normalize(x(7:10));
                Cq = GenericINS.Cb2n(quaternion);
                [roll, pitch, yaw] = GenericINS.EulerZYX(Cq);

                % Calculate position (add vector from accelerometer to body origin)
                iM = diag([(1/(obj.Rn - x(3))); (1/((obj.Re - x(3))*cos(x(1)))); 1.0]);
                positionLLA = x(1:3) - iM * Cq * GenericINSParameters.ACC_POSITION_B2S;
                positionLLA(3) = -positionLLA(3);

                % Calculate the rotational velocity (remove bias and align to body frame)
                velocityPQR = GenericINSParameters.GYR_ORIENTATION_S2B*(obj.imuBuffer(5:7));

                % Calculate the velocity in the navigation frame taking the current rotational rate into account
                velocityNED = x(4:6) - Cq * cross(velocityPQR - Cq' * (obj.omegaEarth * [cos(x(1)); 0; -sin(x(1))]), GenericINSParameters.ACC_POSITION_B2S);

                % Calculate translational body velocity
                velocityUVW = Cq' * velocityNED;
            end
        end
        function diagP = GetDiagP(obj)
            %GenericINS.GetDiagP Get the diagonal of the covariance matrix P. This is the covariance matrix of the state estimation w.r.t. the accelerometer position.
            % 
            % RETURN
            % diagP ... The 15x1 vector that contains the diagonal elements of P.
            Px = obj.srssukf.GetCovariance();
            diagP = diag(Px);
        end
        function Reset(obj)
            %GenericINS.Reset Reset the sensor fusion to the default inital state (NaN).
            obj.initialized = false;
            obj.imuBuffer = nan(7,1);
            obj.gnssBuffer = nan(4,1);
            obj.dvlBuffer = nan(4,1);
            obj.ahrsBuffer = nan(4,1);
            obj.gnssTrigger = false;
            obj.dvlTrigger = false;
            obj.ahrsTrigger = false;
        end
    end


    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % STATIC FUNCTIONS
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    methods(Static)
        function y = Normalize(u)
            %GenericINS.Normalize Safe normalization.
            % 
            % PARAMETERS
            % u ... Input vector. Must contain at least one element!
            % 
            % RETURN
            % y ... Normalized output vector. If input vector is close to zero, a default vector will be returned.
            len = norm(u);
            if(len < 1e-6)
                u = ones(size(u));
                len = norm(u);
            end
            y = (1.0 / len) * u;
        end
        function C = Cb2n(q)
            %GenericINS.Cb2n Calculate the rotation matrix for a given unit quaternion.
            % 
            % PARAMETERS
            % q ... 4x1 normalized unit quaternion (b-frame to n-frame).
            % 
            % RETURN
            % C ... 3x3 rotation matrix (b-frame to n-frame).
            q = reshape(q, [4 1]);
            q0q0 = q(1) * q(1);
            q1q1 = q(2) * q(2);
            q2q2 = q(3) * q(3);
            q3q3 = q(4) * q(4);
            q1q2 = q(2) * q(3);
            q0q3 = q(1) * q(4);
            q1q3 = q(2) * q(4);
            q0q2 = q(1) * q(3);
            q2q3 = q(3) * q(4);
            q0q1 = q(1) * q(2);
            C = [
                q0q0+q1q1-q2q2-q3q3 q1q2+q1q2-q0q3-q0q3 q1q3+q1q3+q0q2+q0q2;
                q1q2+q1q2+q0q3+q0q3 q0q0-q1q1+q2q2-q3q3 q2q3+q2q3-q0q1-q0q1;
                q1q3+q1q3-q0q2-q0q2 q2q3+q2q3+q0q1+q0q1 q0q0-q1q1-q2q2+q3q3;
            ];
        end
        function [roll, pitch, yaw] = EulerZYX(R)
            %GenericINS.EulerZYX Calculate the corresponding euler angles (zyx-convention) for a given rotation matrix (b-frame to n-frame).
            % 
            % PARAMETERS
            % R ... 3x3 Rotation matrix (b-frame to n-frame).
            % 
            % RETURN
            % roll  ... Resulting roll angle in radians.
            % pitch ... Resulting pitch angle in radians.
            % yaw   ... Resulting yaw angle in radians.
            R = reshape(R, [3 3]);
            roll = atan2(R(3,2), R(3,3));
            pitch = -real(asin(min(max(R(3,1),-1.0),1.0)));
            yaw = atan2(R(2,1), R(1,1));
        end
        function [Omega, Rn, Re, R0] = WGS84(latitude)
            %GenericINS.WGS84 Get information about the WGS84 earth reference model for a given latitude in radians.
            % 
            % PARAMETERS
            % latitude ... Latitude in radians.
            % 
            % RETURN
            % Omega ... Angular rate [rad/s] of the earth w.r.t. the inertial frame.
            % Rn    ... Northern earth radius [m].
            % Re    ... Eastern earth radius [m].
            % R0    ... Mean earth radius [m].
            a = 6378137.0;
            ee = 0.0066943799901410724656084644; % e*e, e=0.081819190842621
            s = sin(latitude);
            invRoot = 1.0 / sqrt(1 - ee * s * s);
            Re = a * invRoot;
            Rn = Re * (1-ee) * (invRoot*invRoot);
            R0 = sqrt(Re * Rn);
            Omega = 7.292115e-5;
        end
        function g = Gravity(h, latitude, R0)
            %GenericINS.Gravity Compute the local gravity g [m/s^2] for a given position using the WGS84 earth reference model.
            % 
            % PARAMETERS
            % h        ... Height [m] w.r.t. the WGS84 reference ellipsoid (positive downwards).
            % latitude ... Geographical latitude [rad] for the position of interest.
            % R0       ... Mean earth radius [m] (WGS84).
            % 
            % RETURN
            % g ... scalar local gravity [m/s^2].
            s = sin(latitude);
            s2 = sin(2 * latitude);
            R = R0 / (R0 - h);
            g = 9.780318 * (1 + 5.3024e-3 * s*s + 5.9e-6 * s2*s2) * R * R;
        end
        function [lat, lon] = LatLon(phi, lambda)
            %GenericINS.LatLon Convert phi [rad] and lambda [rad] to latitude [rad] and longitude [rad] with latitude being in range [-pi/2, +pi/2] and longitude being in range [-pi; +pi).
            % 
            % PARAMETERS
            % phi    ... Input angle for latitude in radians.
            % lambda ... Input angle for longitude in radians.
            % 
            % RETURN
            % lat ... Output angle for latitude [rad] being in range [-pi/2, +pi/2].
            % lon ... Output angle for longitude [rad] being in range [-pi, +pi).
            const_pi_half = 0.5 * pi;
            const_pi_2 = (pi + pi);

            % Make sure that inputs are scalars
            lambda = reshape(lambda, [1 1]);
                    phi = reshape(phi, [1 1]);

            % Convert phi to symmetric angle [-pi, pi)
            phi = double(mod(phi, const_pi_2));
            phi = phi - double(phi >= pi) * const_pi_2;

            % Check for (phi > pi/2) and (phi < -pi/2)
            if(phi > const_pi_half)
                phi = pi - phi;
                lambda = lambda + pi;
            elseif(phi < -const_pi_half)
                phi = -pi - phi;
                lambda = lambda + pi;
            end

            % Convert lambda to symmetric angle [-pi, pi)
            lambda = double(mod(lambda, const_pi_2));
            lambda = lambda - double(lambda >= pi) * const_pi_2;

            % Set output
            lat = phi;
            lon = lambda;
        end
        function xOut = ProcessModel(x, w, u, Ts, optArg)
            % Output size
            xOut = x;

            % Inputs: remove estimated biases and align to b-frame
            f_ib = GenericINSParameters.ACC_ORIENTATION_S2B * (u(1:3) + w(1:3) - x(11:13));
            w_ib = GenericINSParameters.GYR_ORIENTATION_S2B * (u(4:6) + w(4:6));

            % Current attitude
            x(7:10) = GenericINS.Normalize(x(7:10));
            DCM_b2n = GenericINS.Cb2n(x(7:10));
            DCM_n2b = DCM_b2n';

            % Predict position
            xOut(1) = x(1) + Ts * (x(4) / (optArg.Rn - x(3)));
            xOut(2) = x(2) + Ts * (x(5) / ((optArg.Re - x(3)) * cos(x(1))));
            xOut(3) = x(3) + Ts * (x(6));

            % lat/lon range conversion
            [lat, lon] = GenericINS.LatLon(xOut(1), xOut(2));
            xOut(1) = lat;
            xOut(2) = lon;

            % Predict velocity
            w_ie = [optArg.omegaEarth * cos(x(1)); 0; -optArg.omegaEarth * sin(x(1))];
            w_en = [(x(5) / ((optArg.Re - x(3)))); -(x(4) / (optArg.Rn - x(3))); -(x(5) / (optArg.Re - x(3)))*tan(x(1))];
            coriolis = cross(2 * w_ie + w_en, x(4:6));
            xOut(4:6) = x(4:6) + Ts * (DCM_b2n * f_ib - coriolis + optArg.localGravity);

            % Predict attitude
            w_nb = w_ib - DCM_n2b * (w_ie + w_en);
            Omega = [0 -w_nb(1) -w_nb(2) -w_nb(3); w_nb(1) 0 w_nb(3) -w_nb(2); w_nb(2) -w_nb(3) 0 w_nb(1); w_nb(3) w_nb(2) -w_nb(1) 0];
            xOut(7:10) = (eye(4)*(1 - Ts*Ts*(w_nb(1)*w_nb(1) + w_nb(2)*w_nb(2) + w_nb(3)*w_nb(3))/8) + 0.5*Ts*Omega) * x(7:10);
            xOut(7:10) = GenericINS.Normalize(xOut(7:10));

            % Predict inertial biases (Random Walk)
            xOut(11:13) = x(11:13) + w(7:9);
        end
        function yOut = SensorModelGNSS(x, ~, optArg)
            % Obtain position and attitude from state x
            phi = x(1);
            lambda = x(2);
            h = x(3);
            q = GenericINS.Normalize(x(7:10));

            % The position of the GPS-antenna
            M = diag([(1/(optArg.Rn - h)); (1/((optArg.Re - h) * cos(phi))); 1.0]);
            p = [phi; lambda; h] + M * GenericINS.Cb2n(q) * (GenericINSParameters.GNSS_POSITION_B2S - GenericINSParameters.ACC_POSITION_B2S);
            yOut = [p(1); p(2); -p(3)];

            % lat/lon range conversion
            [lat, lon] = GenericINS.LatLon(yOut(1), yOut(2));
            yOut(1) = lat;
            yOut(2) = lon;
        end
        function yOut = SensorModelDVL(x, ~, optArg)
            % Direction cosine matrices from current quaternion
            DCM_b2n = GenericINS.Cb2n(GenericINS.Normalize(x(7:10)));
            DCM_n2b = DCM_b2n';

            % Angular rate of body with respect to the earth
            w_ie = optArg.omegaEarth * [cos(x(1)); 0; -sin(x(1))];
            w_eb = GenericINSParameters.GYR_ORIENTATION_S2B * (reshape(optArg.gyr, [3 1])) - DCM_n2b * w_ie;

            % Additional velocity due to angular speed + sensor alignment in b-frame
            v_b = cross(w_eb, GenericINSParameters.DVL_POSITION_B2S - GenericINSParameters.ACC_POSITION_B2S);

            % Transform additional b-frame velocity to n-frame and add current prediction of v_eb (n-frame)
            v_n = x(4:6) + DCM_b2n * v_b;

            % Finally, transform velocity to sensor frame of the DVL
            y_s = GenericINSParameters.DVL_ORIENTATION_B2S * DCM_n2b * v_n;
            yOut = y_s;
        end
        function yOut = SensorModelAHRS(x, ~, ~)
            % Get attitude from state vector
            q = GenericINS.Normalize(x(7:10));

            % Transform to roll, pitch, yaw
            C = GenericINS.Cb2n(q);
            [roll,pitch,yaw] = GenericINS.EulerZYX(C);

            % Output
            yOut = [roll; pitch; yaw];
        end
    end


    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % PRIVATE MEMBER ATTRIBUTES
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    properties(Access=private)
        % State estimator
        srssukf      % Square-root spherical simplex unscented kalman filter for state estimation.
        initialized  % True if initialized, false otherwise.

        % WGS84 properties
        omegaEarth   % [rad] Angular speed of the earth.
        Rn           % [m] Northern earth radius.
        Re           % [m] Eastern earth radius.
        R0           % [m] Mean earth radius.
        localGravity % [m/s^2] Local gravity.

        % Sensor buffers
        imuBuffer    % 7x1 IMU buffer (t, ax, ay, az, wx, wy, wz)
        gnssBuffer    % 4x1 GPS buffer (t, lat, lon, alt).
        dvlBuffer    % 4x1 DVL buffer (t, vx, vy, vz).
        ahrsBuffer   % 4x1 AHRS buffer (t, roll, pitch, yaw).

        % Sensor triggers
        gnssTrigger   % True if new GPS data available, false otherwise.
        dvlTrigger   % True if new DVL data available, false otherwise.
        ahrsTrigger  % True if new AHRS data available, false otherwise.
    end


    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % PRIVATE MEMBER FUNCTIONS
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    methods(Access=private)
        function Initialize(obj, p_0, v_0, q_0, ba_0)
            %GenericINS.Initialize Initialize the sensor fusion filter using initial GPS position p_0, velocity v_0, unit quaternion and inertial biases.
            % 
            % PARAMETERS
            % p_0 ... 3x1 vector (lat [rad], lon [rad], height [m]). p_0 denotes an initial GPS position. p_0(3) must be positive downwards. This position will
            %         be transformed to the accelerometer using the initial attitude quaternion.
            % v_0 ... 3x1 vector (vNorth [m/s], vEast [m/s], vDown [m/s]).
            % q_0 ... 4x1 vector (q0,q1,q2,q3) where [q1;q2;q3] denotes the vector part of the quaternion. q_0 will be normalized.
            % ba_0 .. 3x1 vector Accelerometer bias (bax [m/s^2], bay [m/s^2], baz [m/s^2]).

            % Set initial state
            x0 = obj.srssukf.GetState();
            x0(1:3) = reshape(p_0, [3 1]);
            x0(4:6) = reshape(v_0, [3 1]);
            x0(7:10) = reshape(q_0, [4 1]);
            x0(11:13) = reshape(ba_0, [3 1]);

            % Make sure that attitude is a unit quaternion
            x0(7:10) = GenericINS.Normalize(x0(7:10));

            % Accelerometer position is the initial position
            [obj.omegaEarth, obj.Rn, obj.Re, obj.R0] = GenericINS.WGS84(x0(1));
            r = GenericINS.Cb2n(x0(7:10)) * (GenericINSParameters.ACC_POSITION_B2S - GenericINSParameters.GNSS_POSITION_B2S);
            x0(2) = x0(2) + r(2)/((obj.Re - x0(3))*cos(x0(1)));
            x0(1) = x0(1) + r(1)/(obj.Rn - x0(3));
            x0(3) = x0(3) + r(3);

            % Initialize WGS84 properties
            [obj.omegaEarth, obj.Rn, obj.Re, obj.R0] = GenericINS.WGS84(x0(1));
            obj.localGravity = [0.0; 0.0; GenericINS.Gravity(x0(3), x0(1), obj.R0)];

            % Set uncertainties
            Qa  = diag([GenericINSParameters.ACC_VARIANCE_X; GenericINSParameters.ACC_VARIANCE_Y; GenericINSParameters.ACC_VARIANCE_Z]);
            Qw  = diag([GenericINSParameters.GYR_VARIANCE_X; GenericINSParameters.GYR_VARIANCE_Y; GenericINSParameters.GYR_VARIANCE_Z]);
            Qba = diag([GenericINSParameters.ACC_BIAS_VARIANCE_X; GenericINSParameters.ACC_BIAS_VARIANCE_Y; GenericINSParameters.ACC_BIAS_VARIANCE_Z]);
            Q = blkdiag(Qa, Qw, Qba);
            P0 = diag(GenericINSParameters.DIAG_P0);
            Sx = chol(P0, 'lower');
            Sw = chol(Q, 'lower');
            S0 = blkdiag(Sx, Sw);

            % Actual filter initialization
            obj.srssukf.Initialize(x0, S0);
        end
        function Predict(obj, imuData, Ts)
            %GenericINS.Predict Perform a time-update step using inertial measurement data.
            % 
            % PARAMETERS
            % imuData ... 6x1 vector including 3x1 acceleration [m/s^2] and 3x1 angular rate [rad/s] measurements.
            % Ts      ... Elapsed time [s] to previous Predict() call.

            % Update WGS84 parameters
            x = obj.srssukf.GetState();
            [obj.omegaEarth, obj.Rn, obj.Re, obj.R0] = GenericINS.WGS84(x(1));
            obj.localGravity = [0.0; 0.0; GenericINS.Gravity(x(3), x(1), obj.R0)];

            % Set optional arguments
            optArg = struct('Rn',obj.Rn,'Re',obj.Re,'omegaEarth',obj.omegaEarth,'localGravity',obj.localGravity);

            % Perform the actual prediction
            obj.srssukf.Predict(@GenericINS.ProcessModel, Ts, imuData, [], optArg);
        end
        function ret = UpdateGNSS(obj, gnssData)
            %GenericINS.UpdateGNSS Perform a measurement update step for a GNSS measurement gps (lat [rad], lon [rad], h [m]). Note that the height h of a GNSS message is positive upwards.
            % 
            % PARAMETERS
            % gnssData ... 3x1 vector, (lat [rad]; lon [rad]; h [m]).
            % 
            % RETURN
            % ret ... 1x1 scalar squared innovation (gps - y_mdl)'*(gps - y_mdl) in meters.

            % Specify indices for angles
            idxAngle = int32(2);

            % Calculate uncertainty: [m]->[rad] for lat,lon
            x = obj.srssukf.GetState();
            sqrtR = diag([(GenericINSParameters.GNSS_STANDARD_DEVIATION_NORTH / (obj.Rn - x(3))); (GenericINSParameters.GNSS_STANDARD_DEVIATION_EAST / ((obj.Re - x(3)) * cos(x(1)))); GenericINSParameters.GNSS_STANDARD_DEVIATION_DOWN]);

            % Set optional arguments
            optArg = struct('Rn',obj.Rn,'Re',obj.Re);

            % Perform the actual measurement update
            [x, innov] = obj.srssukf.Update(@GenericINS.SensorModelGNSS, 1, gnssData, idxAngle, sqrtR, optArg);

            % Compute difference of measurement and estimation (squared distance)
            iM = diag([(obj.Rn - x(3)); ((obj.Re - x(3)) * cos(x(1))); 1.0]);
            delta = iM * innov;
            ret = delta' * delta;
        end
        function UpdateDVL(obj, dvlData, gyr)
            %GenericINS.UpdateDVL Perform a measurement update step for a DVL measurement dvlData [m/s]. gyr denotes the current angular rate from the gyroscope.
            % 
            % PARAMETERS
            % dvl ... 3x1 vector, DVL body velocity measurements (bottom track).
            % gyr ... 3x1 vector, current angular rate measurement [rad/s].

            % Set optional arguments
            optArg = struct('omegaEarth',obj.omegaEarth,'gyr',gyr);

            % Perform the actual measurement update
            sqrtR = diag([GenericINSParameters.DVL_STANDARD_DEVIATION_X; GenericINSParameters.DVL_STANDARD_DEVIATION_Y; GenericINSParameters.DVL_STANDARD_DEVIATION_Z]);
            obj.srssukf.Update(@GenericINS.SensorModelDVL, 2, dvlData, int32([]), sqrtR, optArg);
        end
        function UpdateAHRS(obj, ahrsData)
            %GenericINS.UpdateAHRS Perform a measurement update step for AHRS measurement ahrsData [rad].
            % 
            % PARAMETERS
            % ahrsData ... Roll, pitch and yaw angles in radians.

            % Specify indices for angles
            idxAngle = int32(3);

            % Perform the actual measurement update
            sqrtR = diag([GenericINSParameters.AHRS_STANDARD_DEVIATION_ROLL; GenericINSParameters.AHRS_STANDARD_DEVIATION_PITCH; GenericINSParameters.AHRS_STANDARD_DEVIATION_YAW]);
            obj.srssukf.Update(@GenericINS.SensorModelAHRS, 3, ahrsData, idxAngle, sqrtR, []);
        end
    end
end


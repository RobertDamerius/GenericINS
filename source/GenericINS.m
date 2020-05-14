% GenericINS.m
% 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Version     Author                 Changes
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 20200207    Robert Damerius        Initial release.
% 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
classdef GenericINS < handle
    methods(Static)
        function code = GenerateFunction(functionName, positionMeasurements, velocityMeasurements, orientationMeasurements)
            %GenericINS.GenerateFunction Generate a MATLAB function that implements a complete INS algorithm for a specific sensor configuration.
            % 
            % PARAMETERS
            % functionName           ... The name of the function. The function will be saved as [functionName '.m'].
            % positionMeasurements   ... N-by-1 vector with N being the number of position measurement sensors. Each element in the vector indicates the dimension. Possible values are:
            %                              1: Position sensor that measures only altitude.
            %                              2: Position sensor that measures only latitude and longitude.
            %                              3: Position sensor that measures latitude, longitude and altitude.
            % velocityMeasurements   ... N-by-1 vector with N being the number of velocity measurement sensors. Each element in the vector indicates the dimension. Possible values are:
            %                              1: Velocity sensor that measures only in z-direction.
            %                              2: Velocity sensor that measures only in x- and y-direction.
            %                              3: Velocity sensor that measures in all three directions.
            % orientationMeasurements ... N-by-1 vector with N being the number of orientation measurement sensors. Each element in the vector indicates the dimension. Possible values are:
            %                              1: Orientation sensor that measures only yaw.
            %                              2: Orientation sensor that measures only roll and pitch.
            %                              3: Orientation sensor that measures roll, pitch and yaw.
            % 
            % RETURN
            % code ... The generated code as a string.

            % Check for correct inputs
            positionMeasurements = int32(positionMeasurements);
            velocityMeasurements = int32(velocityMeasurements);
            orientationMeasurements = int32(orientationMeasurements);
            numPositionMeasurements = int32(length(positionMeasurements));
            numVelocityMeasurements = int32(length(velocityMeasurements));
            numOrientationMeasurements = int32(length(orientationMeasurements));
            positionMeasurements = reshape(positionMeasurements, int32(1), numPositionMeasurements);
            velocityMeasurements = reshape(velocityMeasurements, int32(1), numVelocityMeasurements);
            orientationMeasurements = reshape(orientationMeasurements, int32(1), numOrientationMeasurements);
            assert(0 == (sum(positionMeasurements < 1) + sum(positionMeasurements > 3)), 'GenericINS.GenerateFunction(): "positionMeasurements" must only contain integer values {1, 2, 3}!');
            assert(0 == (sum(velocityMeasurements < 1) + sum(velocityMeasurements > 3)), 'GenericINS.GenerateFunction(): "velocityMeasurements" must only contain integer values {1, 2, 3}!');
            assert(0 == (sum(orientationMeasurements < 1) + sum(orientationMeasurements > 3)), 'GenericINS.GenerateFunction(): "orientationMeasurements" must only contain integer values {1, 2, 3}!');

            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            % Generate argument list, documentation for function and variable code sections
            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            varCodeSectionA = '';
            varCodeSectionB = '';

            % Default arguments
            docs = ['    %%' functionName ' Generated MATLAB function for a generic inertial navigation system (' num2str(numPositionMeasurements) ' position measurements, ' num2str(numVelocityMeasurements) ' velocity measurements, ' num2str(numOrientationMeasurements) ' orientation measurements).\n'];
            docs = [docs '    %% This function was automatically generated using the command GenericINS.GenerateFunction(''' functionName ''', [' num2str(positionMeasurements) '], [' num2str(velocityMeasurements) '], [' num2str(orientationMeasurements) ']).\n'];
            docs = [docs '    %% \n'];
            docs = [docs '    %% PARAMETERS\n'];
            docs = [docs '    %% initialState            ... 15-by-1 initial state vector to be used when the INS is reset. The elements are as follows:\n'];
            docs = [docs '    %%                              1: latitude     [rad]     Initial latitude of the position of the IMU.\n'];
            docs = [docs '    %%                              2: longitude    [rad]     Initial longitude of the position of the IMU.\n'];
            docs = [docs '    %%                              3: altitude     [m]       Initial altitude of the position of the IMU, positive upwards.\n'];
            docs = [docs '    %%                              4: velocityU    [m/s]     Initial body-fixed velocity in x-direction of the body-frame at the position of the IMU.\n'];
            docs = [docs '    %%                              5: velocityV    [m/s]     Initial body-fixed velocity in y-direction of the body-frame at the position of the IMU.\n'];
            docs = [docs '    %%                              6: velocityW    [m/s]     Initial body-fixed velocity in z-direction of the body-frame at the position of the IMU.\n'];
            docs = [docs '    %%                              7: roll         [rad]     Initial roll angle.\n'];
            docs = [docs '    %%                              8: pitch        [rad]     Initial pitch angle.\n'];
            docs = [docs '    %%                              9: yaw          [rad]     Initial yaw angle.\n'];
            docs = [docs '    %%                             10: biasAccX     [m/(s*s)] Initial acceleration bias in x-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                             11: biasAccY     [m/(s*s)] Initial acceleration bias in y-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                             12: biasAccZ     [m/(s*s)] Initial acceleration bias in z-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                             13: biasGyrX     [rad/s]   Initial angular rate bias around x-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                             14: biasGyrY     [rad/s]   Initial angular rate bias around y-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                             15: biasGyrZ     [rad/s]   Initial angular rate bias around z-axis of IMU sensor frame.\n'];
            docs = [docs '    %% initialStdDev           ... 15-by-1 standard deviation vector that represents the unvertainty of the initial state. It is better to use larger values than\n'];
            docs = [docs '    %%                             too small values for the filter to converge. The elements are as follows:\n'];
            docs = [docs '    %%                              1: latitude     [rad]     Initial standard deviation for initial latitude.\n'];
            docs = [docs '    %%                              2: longitude    [rad]     Initial standard deviation for initial longitude.\n'];
            docs = [docs '    %%                              3: altitude     [m]       Initial standard deviation for initial altitude.\n'];
            docs = [docs '    %%                              4: velocityU    [m/s]     Initial standard deviation for initial body-fixed velocity in x-direction of the body-frame.\n'];
            docs = [docs '    %%                              5: velocityV    [m/s]     Initial standard deviation for initial body-fixed velocity in y-direction of the body-frame.\n'];
            docs = [docs '    %%                              6: velocityW    [m/s]     Initial standard deviation for initial body-fixed velocity in z-direction of the body-frame.\n'];
            docs = [docs '    %%                              7: orientationX           Initial standard deviation for x-component of orientation vector (axis-angle representation).\n'];
            docs = [docs '    %%                              8: orientationY           Initial standard deviation for y-component of orientation vector (axis-angle representation).\n'];
            docs = [docs '    %%                              9: orientationZ           Initial standard deviation for z-component of orientation vector (axis-angle representation).\n'];
            docs = [docs '    %%                             10: biasAccX     [m/(s*s)] Initial standard deviation for acceleration bias in x-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                             11: biasAccY     [m/(s*s)] Initial standard deviation for acceleration bias in y-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                             12: biasAccZ     [m/(s*s)] Initial standard deviation for acceleration bias in z-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                             13: biasGyrX     [rad/s]   Initial standard deviation for angular rate bias around x-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                             14: biasGyrY     [rad/s]   Initial standard deviation for angular rate bias around y-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                             15: biasGyrZ     [rad/s]   Initial standard deviation for angular rate bias around z-axis of IMU sensor frame.\n'];
            docs = [docs '    %% reset                   ... Greater than 0.0 if filter should be reset/reinitialized, 0.0 otherwise. During an initialization the initial state and initial standard deviations are used.\n'];
            docs = [docs '    %% sampletime              ... Discrete sampletime in seconds to be used for forward euler integration. Should be the elapsed time to the latest prediction.\n'];
            docs = [docs '    %% IMU_Data                ... The 7-by-1 vector representing the IMU data where the elements are as follows:\n'];
            docs = [docs '    %%                              1: trigger                Greater than 0.0 if new IMU data is available and should be used in a prediction step, 0.0 otherwise.\n'];
            docs = [docs '    %%                              2: accX         [m/(s*s)] Acceleration in x-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                              3: accY         [m/(s*s)] Acceleration in y-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                              4: accZ         [m/(s*s)] Acceleration in z-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                              5: gyrX         [rad/s]   Angular rate around x-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                              6: gyrY         [rad/s]   Angular rate around y-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                              7: gyrZ         [rad/s]   Angular rate around z-axis of IMU sensor frame.\n'];
            docs = [docs '    %% IMU_StdDev              ... 12-by-1 vector representing the standard deviation for inertial measurements and inertial biases. The elements are as follows:\n'];
            docs = [docs '    %%                              1: accX         [m/(s*s)] Standard deviation for acceleration in x-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                              2: accY         [m/(s*s)] Standard deviation for acceleration in y-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                              3: accZ         [m/(s*s)] Standard deviation for acceleration in z-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                              4: gyrX         [rad/s]   Standard deviation for angular rate around x-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                              5: gyrY         [rad/s]   Standard deviation for angular rate around y-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                              6: gyrZ         [rad/s]   Standard deviation for angular rate around z-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                              7: biasAccX     [m/(s*s)] Standard deviation for acceleration bias in x-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                              8: biasAccY     [m/(s*s)] Standard deviation for acceleration bias in y-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                              9: biasAccZ     [m/(s*s)] Standard deviation for acceleration bias in z-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                             10: biasGyrX     [rad/s]   Standard deviation for angular rate bias around x-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                             11: biasGyrY     [rad/s]   Standard deviation for angular rate bias around y-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                             12: biasGyrZ     [rad/s]   Standard deviation for angular rate bias around z-axis of IMU sensor frame.\n'];
            docs = [docs '    %% IMU_PositionBody2Sensor ... Position of IMU sensor frame origin w.r.t. body frame origin in body frame coordinates (meters).\n'];
            docs = [docs '    %% IMU_DCMBody2Sensor      ... Direction cosine matrix to rotate measurements from body frame to sensor frame.\n'];
            args = 'initialState, initialStdDev, reset, sampletime, IMU_Data, IMU_StdDev, IMU_PositionBody2Sensor, IMU_DCMBody2Sensor';

            % Position measurements: additional arguments, documentation and variable code sections
            for n = int32(1):numPositionMeasurements
                dim = positionMeasurements(n);
                arg_Pn_Data = ['P' num2str(n) '_Data'];
                arg_Pn_StdDev = ['P' num2str(n) '_StdDev'];
                arg_Pn_PositionBody2Sensor = ['P' num2str(n) '_PositionBody2Sensor'];
                arg_Pn_Buffer = ['P' num2str(n) '_Buffer'];
                args = [args ', ' arg_Pn_Data ', ' arg_Pn_StdDev ', ' arg_Pn_PositionBody2Sensor];
                docs = [docs '    %% ' arg_Pn_Data '                 ... ' num2str(int32(1) + dim) '-by-1 vector representing the position measurements of position sensor ' num2str(n) '. The elements are as follows:\n'];
                docs = [docs '    %%                              1: trigger                Greater than 0.0 if new measurement data is available and should be used in an update step, 0.0 otherwise.\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                              2: altitude     [m]       The altitude measurement.\n'];
                    case 2
                        docs = [docs '    %%                              2: latitude     [rad]     The latitude measurement.\n'];
                        docs = [docs '    %%                              3: longitude    [rad]     The longitude measurement.\n'];
                    case 3
                        docs = [docs '    %%                              2: latitude     [rad]     The latitude measurement.\n'];
                        docs = [docs '    %%                              3: longitude    [rad]     The longitude measurement.\n'];
                        docs = [docs '    %%                              4: altitude     [m]       The altitude measurement.\n'];
                end
                docs = [docs '    %% ' arg_Pn_StdDev '               ... ' num2str(dim) '-by-1 vector representing the standard deviation for the position measurements of position sensor ' num2str(n) '. The elements are as follows:\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                              1: stdDown      [m]       Standard deviation in up/down-direction.\n'];
                    case 2
                        docs = [docs '    %%                              1: stdNorth     [m]       Standard deviation in north direction.\n'];
                        docs = [docs '    %%                              2: stdEast      [m]       Standard deviation in east direction.\n'];
                    case 3
                        docs = [docs '    %%                              1: stdNorth     [m]       Standard deviation in north direction.\n'];
                        docs = [docs '    %%                              2: stdEast      [m]       Standard deviation in east direction.\n'];
                        docs = [docs '    %%                              3: stdDown      [m]       Standard deviation in up/down-direction.\n'];
                end
                docs = [docs '    %% ' arg_Pn_PositionBody2Sensor '  ... Position of position sensor ' num2str(n) ' w.r.t. body frame origin in body frame coordinates (meters).\n'];

                % Variable code section A
                varCodeSectionA = [varCodeSectionA '    assert((' num2str(int32(1) + dim) ' == size(' arg_Pn_Data ',1)) && (1 == size(' arg_Pn_Data ',2)) && (' num2str(dim) ' == size(' arg_Pn_StdDev ',1)) && (1 == size(' arg_Pn_StdDev ',2)) && (3 == size(' arg_Pn_PositionBody2Sensor ',1)) && (1 == size(' arg_Pn_PositionBody2Sensor ',2)));\n'];
                varCodeSectionA = [varCodeSectionA '    persistent ' arg_Pn_Buffer '; if(isempty(' arg_Pn_Buffer ') || (' arg_Pn_Data '(1) > 0.0)), ' arg_Pn_Buffer ' = ' arg_Pn_Data '; end\n'];

                % Variable code section B
                varCodeSectionB = [varCodeSectionB '        if(' arg_Pn_Buffer '(1) > 0.0), ' arg_Pn_Buffer '(1) = 0.0; ins.UpdatePosition' num2str(dim) 'D(' arg_Pn_Buffer '(2:end), ' arg_Pn_StdDev ', ' arg_Pn_PositionBody2Sensor '); end\n'];
            end

            % Velocity measurements: additional arguments, documentation and variable code sections
            for n = int32(1):numVelocityMeasurements
                dim = velocityMeasurements(n);
                arg_Vn_Data = ['V' num2str(n) '_Data'];
                arg_Vn_StdDev = ['V' num2str(n) '_StdDev'];
                arg_Vn_PositionBody2Sensor = ['V' num2str(n) '_PositionBody2Sensor'];
                arg_Vn_DCMBody2Sensor = ['V' num2str(n) '_DCMBody2Sensor'];
                arg_Vn_Buffer = ['V' num2str(n) '_Buffer'];
                args = [args ', ' arg_Vn_Data ', ' arg_Vn_StdDev ', ' arg_Vn_PositionBody2Sensor ', ' arg_Vn_DCMBody2Sensor];
                docs = [docs '    %% ' arg_Vn_Data '                 ... ' num2str(int32(1) + dim) '-by-1 vector representing the velocity measurements of velocity sensor ' num2str(n) '. The elements are as follows:\n'];
                docs = [docs '    %%                              1: trigger                Greater than 0.0 if new measurement data is available and should be used in an update step, 0.0 otherwise.\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                              2: velocityZ    [m/s]     The velocity measurement in z-direction of sensor frame.\n'];
                    case 2
                        docs = [docs '    %%                              2: velocityX    [m/s]     The velocity measurement in x-direction of sensor frame.\n'];
                        docs = [docs '    %%                              3: velocityY    [m/s]     The velocity measurement in y-direction of sensor frame.\n'];
                    case 3
                        docs = [docs '    %%                              2: velocityX    [m/s]     The velocity measurement in x-direction of sensor frame.\n'];
                        docs = [docs '    %%                              3: velocityY    [m/s]     The velocity measurement in y-direction of sensor frame.\n'];
                        docs = [docs '    %%                              4: velocityZ    [m/s]     The velocity measurement in z-direction of sensor frame.\n'];
                end
                docs = [docs '    %% ' arg_Vn_StdDev '               ... ' num2str(dim) '-by-1 vector representing the standard deviation for the velocity measurements of velocity sensor ' num2str(n) '. The elements are as follows:\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                              1: stdZ         [m/s]     Standard deviation in z-direction.\n'];
                    case 2
                        docs = [docs '    %%                              1: stdX         [m/s]     Standard deviation in x-direction.\n'];
                        docs = [docs '    %%                              2: stdY         [m/s]     Standard deviation in y-direction.\n'];
                    case 3
                        docs = [docs '    %%                              1: stdX         [m/s]     Standard deviation in x-direction.\n'];
                        docs = [docs '    %%                              2: stdY         [m/s]     Standard deviation in y-direction.\n'];
                        docs = [docs '    %%                              3: stdZ         [m/s]     Standard deviation in z-direction.\n'];
                end
                docs = [docs '    %% ' arg_Vn_PositionBody2Sensor '  ... Position of velocity sensor ' num2str(n) ' w.r.t. body frame origin in body frame coordinates (meters).\n'];
                docs = [docs '    %% ' arg_Vn_DCMBody2Sensor '       ... Direction cosine matrix to rotate vectors from body frame to sensor frame.\n'];

                % Variable code section A
                varCodeSectionA = [varCodeSectionA '    assert((' num2str(int32(1) + dim) ' == size(' arg_Vn_Data ',1)) && (1 == size(' arg_Vn_Data ',2)) && (' num2str(dim) ' == size(' arg_Vn_StdDev ',1)) && (1 == size(' arg_Vn_StdDev ',2)) && (3 == size(' arg_Vn_PositionBody2Sensor ',1)) && (1 == size(' arg_Vn_PositionBody2Sensor ',2)) && (3 == size(' arg_Vn_DCMBody2Sensor ',1)) && (3 == size(' arg_Vn_DCMBody2Sensor ',2)));\n'];
                varCodeSectionA = [varCodeSectionA '    persistent ' arg_Vn_Buffer '; if(isempty(' arg_Vn_Buffer ') || (' arg_Vn_Data '(1) > 0.0)), ' arg_Vn_Buffer ' = ' arg_Vn_Data '; end\n'];

                % Variable code section B
                varCodeSectionB = [varCodeSectionB '        if(' arg_Vn_Buffer '(1) > 0.0), ' arg_Vn_Buffer '(1) = 0.0; ins.UpdateVelocity' num2str(dim) 'D(' arg_Vn_Buffer '(2:end), ' arg_Vn_StdDev ', ' arg_Vn_PositionBody2Sensor ', ' arg_Vn_DCMBody2Sensor '); end\n'];
            end

            % Orientation measurements: additional arguments, documentation and variable code sections
            for n = int32(1):numOrientationMeasurements
                dim = orientationMeasurements(n);
                arg_On_Data = ['O' num2str(n) '_Data'];
                arg_On_StdDev = ['O' num2str(n) '_StdDev'];
                arg_On_DCMBody2Sensor = ['O' num2str(n) '_DCMBody2Sensor'];
                arg_On_Buffer = ['O' num2str(n) '_Buffer'];
                args = [args ', ' arg_On_Data ', ' arg_On_StdDev ', ' arg_On_DCMBody2Sensor];
                docs = [docs '    %% ' arg_On_Data '                 ... ' num2str(int32(1) + dim) '-by-1 vector representing the orientation measurements of orientation sensor ' num2str(n) '. Note that orientation measurements must only be used for applications with small roll and pitch angles. The elements are as follows:\n'];
                docs = [docs '    %%                              1: trigger                Greater than 0.0 if new measurement data is available and should be used in an update step, 0.0 otherwise.\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                              2: yaw          [rad]     The yaw angle measurement (ZYX-convention).\n'];
                    case 2
                        docs = [docs '    %%                              2: roll         [rad]     The roll angle measurement (ZYX-convention).\n'];
                        docs = [docs '    %%                              3: pitch        [rad]     The pitch angle measurement (ZYX-convention).\n'];
                    case 3
                        docs = [docs '    %%                              2: roll         [rad]     The roll angle measurement (ZYX-convention).\n'];
                        docs = [docs '    %%                              3: pitch        [rad]     The pitch angle measurement (ZYX-convention).\n'];
                        docs = [docs '    %%                              4: yaw          [rad]     The yaw angle measurement (ZYX-convention).\n'];
                end
                docs = [docs '    %% ' arg_On_StdDev '               ... ' num2str(dim) '-by-1 vector representing the standard deviation for the orientation measurements of orientation sensor ' num2str(n) '. The elements are as follows:\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                              1: stdYaw       [rad]     Standard deviation for yaw angle.\n'];
                    case 2
                        docs = [docs '    %%                              1: stdRoll      [rad]     Standard deviation for roll angle.\n'];
                        docs = [docs '    %%                              2: stdPitch     [rad]     Standard deviation for pitch angle.\n'];
                    case 3
                        docs = [docs '    %%                              1: stdRoll      [rad]     Standard deviation for roll angle.\n'];
                        docs = [docs '    %%                              2: stdPitch     [rad]     Standard deviation for pitch angle.\n'];
                        docs = [docs '    %%                              3: stdYaw       [rad]     Standard deviation for yaw angle.\n'];
                end
                docs = [docs '    %% ' arg_On_DCMBody2Sensor '       ... Direction cosine matrix to rotate vectors from body frame to sensor frame.\n'];

                % Variable code section A
                varCodeSectionA = [varCodeSectionA '    assert((' num2str(int32(1) + dim) ' == size(' arg_On_Data ',1)) && (1 == size(' arg_On_Data ',2)) && (' num2str(dim) ' == size(' arg_On_StdDev ',1)) && (1 == size(' arg_On_StdDev ',2)) && (3 == size(' arg_On_DCMBody2Sensor ',1)) && (3 == size(' arg_On_DCMBody2Sensor ',2)));\n'];
                varCodeSectionA = [varCodeSectionA '    persistent ' arg_On_Buffer '; if(isempty(' arg_On_Buffer ') || (' arg_On_Data '(1) > 0.0)), ' arg_On_Buffer ' = ' arg_On_Data '; end\n'];

                % Variable code section B
                varCodeSectionB = [varCodeSectionB '        if(' arg_On_Buffer '(1) > 0.0), ' arg_On_Buffer '(1) = 0.0; ins.UpdateOrientation' num2str(dim) 'D(' arg_On_Buffer '(2:end), ' arg_On_StdDev ', ' arg_On_DCMBody2Sensor '); end\n'];
            end

            % Return values
            docs = [docs '    %% \n    %% RETURN'];
            docs = [docs '\n    %% valid                      ... A scalar boolean that indicates if the motion state is valid or not. It is valid if the INS was initialized and has performed at least one prediction.'];
            docs = [docs '\n    %% positionLLA                ... Geographic position of the body frame, [lat (rad); lon (rad); alt (m, positive upwards)].'];
            docs = [docs '\n    %% orientationQuaternionWXYZ  ... Unit quaternion describing a rotation from body frame to navigation frame, [qw; qx; qy; qz], where qw indicates the scalar part of the quaternion.'];
            docs = [docs '\n    %% orientationRollPitchYaw    ... Euler angles according to the ZYX-convention.'];
            docs = [docs '\n    %% velocityNED                ... Translational velocity [m/s] of the body frame given in navigation frame coordinates, excluding earth rotation and transport rate.'];
            docs = [docs '\n    %% velocityUVW                ... Translational velocity [m/s] of the body frame given in body frame coordinates, excluding earth rotation and transport rate.'];
            docs = [docs '\n    %% velocityPQR                ... Rotational velocity [rad/s] of the body frame with respect to the navigation frame, excluding earth rotation and transport rate.'];
            docs = [docs '\n    %% courseOverGround           ... Course over ground angle in radians.'];
            docs = [docs '\n    %% speedOverGround            ... Speed over ground in meters per second.'];
            docs = [docs '\n    %% angleOfAttack              ... Angle of attack in radians.'];
            docs = [docs '\n    %% sideSlipAngle              ... Side slip angle in radians.'];

            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            % Generate the final function code
            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            code = ['function [valid, positionLLA, orientationQuaternionWXYZ, orientationRollPitchYaw, velocityNED, velocityUVW, velocityPQR, courseOverGround, speedOverGround, angleOfAttack, sideSlipAngle] = ' functionName '(' args ')\n' docs '\n'];
            code = [code '    assert((15 == size(initialState,1)) && (1 == size(initialState,2)) && (15 == size(initialStdDev,1)) && (1 == size(initialStdDev,2)) && isscalar(reset) && isscalar(sampletime) && (7 == size(IMU_Data,1)) && (1 == size(IMU_Data,2)) && (3 == size(IMU_PositionBody2Sensor,1)) && (1 == size(IMU_PositionBody2Sensor,2)) && (3 == size(IMU_DCMBody2Sensor,1)) && (3 == size(IMU_DCMBody2Sensor,2)));\n'];
            code = [code '    persistent ins; if(isempty(ins)), ins = GenericINS(); end\n'];
            code = [code '    persistent imuStdDevBuffer; if(isempty(imuStdDevBuffer)), imuStdDevBuffer = IMU_StdDev; end\n\n'];
            code = [code varCodeSectionA '\n'];
            code = [code '    imuStdDevChanged = (sum(abs(imuStdDevBuffer - IMU_StdDev) > 100*eps) > 0.0);\n'];
            code = [code '    if(reset > 0.0)\n'];
            code = [code '        ins.Initialize(initialState(1:3), initialState(4:6), initialState(7:9), initialState(10:12), initialState(13:15), initialStdDev(1:3), initialStdDev(4:6), initialStdDev(7:9), initialStdDev(10:12), initialStdDev(13:15), IMU_StdDev(1:3), IMU_StdDev(4:6), IMU_StdDev(7:9), IMU_StdDev(10:12));\n'];
            code = [code '    elseif(imuStdDevChanged)\n'];
            code = [code '        ins.ResetInertialStandardDeviation(IMU_StdDev(1:3), IMU_StdDev(4:6), IMU_StdDev(7:9), IMU_StdDev(10:12));\n'];
            code = [code '    end\n'];
            code = [code '    imuStdDevBuffer = IMU_StdDev;\n'];
            code = [code '    if(IMU_Data(1) > 0.0)\n'];
            code = [code '        %% Motion prediction\n'];
            code = [code '        ins.Predict(IMU_Data(2:7), sampletime, IMU_DCMBody2Sensor, IMU_PositionBody2Sensor);\n\n'];
            code = [code '        %% Sensor updates\n'];
            code = [code varCodeSectionB];
            code = [code '    end\n'];
            code = [code '    [valid, positionLLA, orientationQuaternionWXYZ, orientationRollPitchYaw, velocityNED, velocityUVW, velocityPQR, courseOverGround, speedOverGround, angleOfAttack, sideSlipAngle] = ins.GetMotionState();\n'];
            code = [code 'end\n\n'];

            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            % Write code to a text file
            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            filename = [functionName '.m'];
            fileID = fopen(filename,'w');
            assert(-1 ~= fileID, ['GenericINS.GenerateFunction(): Could not open file "' filename '"!']);
            fprintf(fileID, code);
            fclose(fileID);
        end
    end
    methods
        function obj = GenericINS()
            % Initialize private properties
            obj.valid = false;
            obj.initialized = false;
            obj.x = zeros(GenericINS.DIM_L, 1);
            obj.S = zeros(GenericINS.DIM_LS);
            obj.Xi = zeros(GenericINS.DIM_L, GenericINS.NUM_SP);
            obj.dX = zeros(GenericINS.DIM_XS, GenericINS.NUM_SP);
            obj.spU2D = false;
            obj.gyrRaw = zeros(3, 1);
            obj.dcmIMUBody2Sensor = eye(3);
            obj.posIMUBody2Sensor = zeros(3, 1);
            obj.omegaEarth = 0.0;
            obj.Rn = 0.0;
            obj.Re = 0.0;
            obj.localGravity = 0.0;
            obj.w0 = 0.1;
            obj.wi = (1.0 - obj.w0) / (double(GenericINS.DIM_LS) + 1.0);
            obj.srw0 = sqrt(obj.w0);
            obj.srwi = sqrt(obj.wi);

            % Initialize vector sequence for weighting matrix Z
            obj.Z = zeros(GenericINS.DIM_LS, GenericINS.NUM_SP);
            obj.Z(1,3) = 1.0 / sqrt(2.0 * obj.wi);
            obj.Z(1,2) = -obj.Z(1,3);

            % Expand vector sequence
            for j = int32(2):GenericINS.DIM_LS
                s = 1.0 / sqrt(double(j * (j + 1)) * obj.wi);
                obj.Z(j, j + 2) = double(j) * s;
                for k = int32(2):(j + 1)
                    obj.Z(j, k) = -s;
                end
            end
        end
        function Initialize(obj, initialPositionLLA, initialVelocityUVW, initialOrientationRollPitchYaw, initialBiasAcc, initialBiasGyr, initialStdPositionLLA, initialStdVelocityUVW, initialStdOrientationVector, initialStdBiasAcc, initialStdBiasGyr, stdAcc, stdGyr, stdAccBias, stdGyrBias)
            %GenericINS.Initialize Initialize or re-initialize the INS.
            % 
            % PARAMETERS
            % initialPositionLLA             ... Initial geographic position at the location of the IMU, [lat (rad); lon (rad); alt (m, positive upwards)].
            % initialVelocityUVW             ... Initial body-fixed velocity at the location of the IMU, [u (m/s); v (m/s); r (m/s)].
            % initialOrientationRollPitchYaw ... Initial orientation of the body frame, [roll (rad); pitch (rad); yaw (rad)].
            % initialBiasAcc                 ... Initial acceleration bias in m/s^2.
            % initialBiasGyr                 ... Initial angular rate bias in rad/s.
            % initialStdPositionLLA          ... Initial standard deviation for the geographic position. Should be an estimate of the standard deviation of initialPositionLLA.
            % initialStdVelocityUVW          ... Initial standard deviation for the body-fixed velocity. Should be an estimate of the standard deviation of initialVelocityUVW.
            % initialStdOrientationVector    ... Initial standard deviation for the orientation vector (axis-angle).
            % initialStdBiasAcc              ... Initial standard deviation for acceleration bias.
            % initialStdBiasGyr              ... Initial standard deviation for angular rate bias.
            % stdAcc                         ... Standard deviation for acceleration in m/s^2 to be used for motion prediction.
            % stdGyr                         ... Standard deviation for angular rate in rad/s to be used for motion prediction.
            % stdAccBias                     ... Standard deviation for acceleration bias in m/s^2 to be used for motion prediction.
            % stdGyrBias                     ... Standard deviation for angular rate bias in rad/s to be used for motion prediction.
            assert((3 == size(initialPositionLLA,1)) && (1 == size(initialPositionLLA,2)) && isa(initialPositionLLA, 'double'), 'GenericINS.Initialize(): "initialPositionLLA" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialVelocityUVW,1)) && (1 == size(initialVelocityUVW,2)) && isa(initialVelocityUVW, 'double'), 'GenericINS.Initialize(): "initialVelocityUVW" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialOrientationRollPitchYaw,1)) && (1 == size(initialOrientationRollPitchYaw,2)) && isa(initialOrientationRollPitchYaw, 'double'), 'GenericINS.Initialize(): "initialOrientationRollPitchYaw" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialBiasAcc,1)) && (1 == size(initialBiasAcc,2)) && isa(initialBiasAcc, 'double'), 'GenericINS.Initialize(): "initialBiasAcc" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialBiasGyr,1)) && (1 == size(initialBiasGyr,2)) && isa(initialBiasGyr, 'double'), 'GenericINS.Initialize(): "initialBiasGyr" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialStdPositionLLA,1)) && (1 == size(initialStdPositionLLA,2)) && isa(initialStdPositionLLA, 'double'), 'GenericINS.Initialize(): "initialStdPositionLLA" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialStdVelocityUVW,1)) && (1 == size(initialStdVelocityUVW,2)) && isa(initialStdVelocityUVW, 'double'), 'GenericINS.Initialize(): "initialStdVelocityUVW" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialStdOrientationVector,1)) && (1 == size(initialStdOrientationVector,2)) && isa(initialStdOrientationVector, 'double'), 'GenericINS.Initialize(): "initialStdOrientationVector" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialStdBiasAcc,1)) && (1 == size(initialStdBiasAcc,2)) && isa(initialStdBiasAcc, 'double'), 'GenericINS.Initialize(): "initialStdBiasAcc" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialStdBiasGyr,1)) && (1 == size(initialStdBiasGyr,2)) && isa(initialStdBiasGyr, 'double'), 'GenericINS.Initialize(): "initialStdBiasGyr" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdAcc,1)) && (1 == size(stdAcc,2)) && isa(stdAcc, 'double'), 'GenericINS.Initialize(): "stdAcc" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdGyr,1)) && (1 == size(stdGyr,2)) && isa(stdGyr, 'double'), 'GenericINS.Initialize(): "stdGyr" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdAccBias,1)) && (1 == size(stdAccBias,2)) && isa(stdAccBias, 'double'), 'GenericINS.Initialize(): "stdAccBias" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdGyrBias,1)) && (1 == size(stdGyrBias,2)) && isa(stdGyrBias, 'double'), 'GenericINS.Initialize(): "stdGyrBias" must be a 3-by-1 vector of type double!');

            % Initial position
            [lat, lon] = GenericINS.LatLon(initialPositionLLA(1), initialPositionLLA(2));
            obj.x(1) = lat;
            obj.x(2) = lon;
            obj.x(3) = initialPositionLLA(3);

            % Initial orientation
            phi = 0.5 * initialOrientationRollPitchYaw(1);
			theta = 0.5 * initialOrientationRollPitchYaw(2);
			psi = 0.5 * initialOrientationRollPitchYaw(3);
			c1 = cos(phi);
			c2 = cos(theta);
			c3 = cos(psi);
			s1 = sin(phi);
			s2 = sin(theta);
			s3 = sin(psi);
			obj.x(7:10) = [c1*c2*c3 + s1*s2*s3; s1*c2*c3 - c1*s2*s3; c1*s2*c3 + s1*c2*s3; c1*c2*s3 - s1*s2*c3];

            % Initial velocity
            DCM_b2n = GenericINS.Cb2n(obj.x(7:10));
            obj.x(4:6) = DCM_b2n * initialVelocityUVW;

            % Initial inertial biases
            obj.x(11:13) = initialBiasAcc;
            obj.x(14:16) = initialBiasGyr;

            % Initial square root covariance matrix
            obj.S = diag([initialStdPositionLLA; DCM_b2n * initialStdVelocityUVW; initialStdOrientationVector; initialStdBiasAcc; initialStdBiasGyr; stdAcc; stdGyr; stdAccBias; stdGyrBias]);

            % Initialize additional INS properties
            [obj.omegaEarth, obj.Rn, obj.Re, R0] = GenericINS.WGS84(obj.x(1));
            obj.localGravity = GenericINS.Gravity(obj.x(3), obj.x(1), R0);
            obj.spU2D = false;
            obj.gyrRaw = zeros(3,1);
            obj.valid = false;
            obj.initialized = true;
        end
        function ResetInertialStandardDeviation(obj, stdAcc, stdGyr, stdAccBias, stdGyrBias)
            %GenericINS.ResetInertialStandardDeviation Reset the standard deviations for the inertial sensors.
            % 
            % PARAMETERS
            % stdAcc            ... Standard deviation for acceleration in m/s^2.
            % stdGyr            ... Standard deviation for angular rate in rad/s.
            % stdAccBias        ... Standard deviation for acceleration bias in m/s^2.
            % stdGyrBias        ... Standard deviation for angular rate bias in rad/s.
            assert((3 == size(stdAcc,1)) && (1 == size(stdAcc,2)) && isa(stdAcc, 'double'), 'GenericINS.Predict(): "stdAcc" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdGyr,1)) && (1 == size(stdGyr,2)) && isa(stdGyr, 'double'), 'GenericINS.Predict(): "stdGyr" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdAccBias,1)) && (1 == size(stdAccBias,2)) && isa(stdAccBias, 'double'), 'GenericINS.Predict(): "stdAccBias" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdGyrBias,1)) && (1 == size(stdGyrBias,2)) && isa(stdGyrBias, 'double'), 'GenericINS.Predict(): "stdGyrBias" must be a 3-by-1 vector of type double!');
            obj.S((GenericINS.DIM_XS + int32(1)):end,(GenericINS.DIM_XS + int32(1)):end) = diag([stdAcc; stdGyr; stdAccBias; stdGyrBias]);
        end
        function Predict(obj, imuData, sampletime, dcmIMUBody2Sensor, posIMUBody2Sensor)
            %GenericINS.Predict Perform a prediction step using inertial measurements. The prediction is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % imuData           ... Inertial measurement data vector: [accX (m/s^2); accY (m/s^2); accZ (m/s^2); gyrX (rad/s); gyrY (rad/s); gyrZ (rad/s)]
            % sampletime        ... Discrete sampletime in seconds to be used for forward euler integration. Should be the elapsed time to the latest prediction.
            % dcmIMUBody2Sensor ... Direction cosine matrix to rotate measurements from body frame to sensor frame.
            % posIMUBody2Sensor ... Position of IMU sensor frame origin w.r.t. body frame origin in body frame coordinates (meters).
            assert((6 == size(imuData,1)) && (1 == size(imuData,2)) && isa(imuData, 'double'), 'GenericINS.Predict(): "imuData" must be a 6-by-1 vector of type double!');
            assert(isscalar(sampletime) && isa(sampletime, 'double'), 'GenericINS.Predict(): "sampletime" must be a scalar of type double!');
            assert((3 == size(dcmIMUBody2Sensor,1)) && (3 == size(dcmIMUBody2Sensor,2)) && isa(dcmIMUBody2Sensor, 'double'), 'GenericINS.Predict(): "dcmIMUBody2Sensor" must be a 3-by-3 matrix of type double!');
            assert((3 == size(posIMUBody2Sensor,1)) && (1 == size(posIMUBody2Sensor,2)) && isa(posIMUBody2Sensor, 'double'), 'GenericINS.Predict(): "posIMUBody2Sensor" must be a 3-by-1 vector of type double!');

            % Do not allow prediction of an uninitialized INS
            if(~obj.initialized)
                return;
            end

            % Generate sigma points
            obj.GenerateSigmaPoints();
            obj.spU2D = true;

            % Propagate through process model
            [obj.omegaEarth, obj.Rn, obj.Re, R0] = GenericINS.WGS84(obj.x(1));
            obj.localGravity = GenericINS.Gravity(obj.x(3), obj.x(1), R0);
            for i = int32(1):GenericINS.NUM_SP
                state = obj.Xi(1:GenericINS.DIM_X,i);
                w = obj.Xi((GenericINS.DIM_X+int32(1)):(GenericINS.DIM_X + GenericINS.DIM_W),i);
                obj.Xi(1:GenericINS.DIM_X,i) = GenericINS.ProcessModelEuler(state, w, imuData, sampletime, dcmIMUBody2Sensor, obj.Rn, obj.Re, obj.omegaEarth, obj.localGravity);
            end

            % Calculate dX = Xi - x
            obj.CalculateDX();

            % Update square root covariance
            [~, Sx_UPPER] = qr((obj.srwi*obj.dX(:,2:GenericINS.NUM_SP))', 0);
            Sx_UPPER = cholupdate(Sx_UPPER, obj.srw0*obj.dX(:,1), '+');
            obj.S(1:GenericINS.DIM_XS,1:GenericINS.DIM_XS) = Sx_UPPER';

            % Save latest IMU data
            obj.gyrRaw = imuData(4:6);
            obj.dcmIMUBody2Sensor = dcmIMUBody2Sensor;
            obj.posIMUBody2Sensor = posIMUBody2Sensor;

            % Make INS data valid
            obj.valid = true;
        end
        function UpdatePosition3D(obj, measurementLatLonAlt, stdNorthEastDown, posBody2Sensor)
            %GenericINS.UpdatePosition3D Perform an observation step for 3D geographic position data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementLatLonAlt ... The measurement vector [lat (rad); lon (rad); alt (m, positive upwards)].
            % stdNorthEastDown     ... The standard deviation in meters, [north; east; down].
            % posBody2Sensor       ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            assert((3 == size(measurementLatLonAlt,1)) && (1 == size(measurementLatLonAlt,2)) && isa(measurementLatLonAlt, 'double'), 'GenericINS.UpdatePosition3D(): "measurementLatLonAlt" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdNorthEastDown,1)) && (1 == size(stdNorthEastDown,2)) && isa(stdNorthEastDown, 'double'), 'GenericINS.UpdatePosition3D(): "stdNorthEastDown" must be a 3-by-1 vector of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.UpdatePosition3D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            obj.UpdatePosition(3, measurementLatLonAlt, stdNorthEastDown, posBody2Sensor);
        end
        function UpdatePosition2D(obj, measurementLatLon, stdNorthEast, posBody2Sensor)
            %GenericINS.UpdatePosition2D Perform an observation step for 2D geographic position data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementLatLon ... The measurement vector [lat (rad); lon (rad)].
            % stdNorthEast      ... The standard deviation in meters, [north; east].
            % posBody2Sensor    ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            assert((2 == size(measurementLatLon,1)) && (1 == size(measurementLatLon,2)) && isa(measurementLatLon, 'double'), 'GenericINS.UpdatePosition2D(): "measurementLatLon" must be a 2-by-1 vector of type double!');
            assert((2 == size(stdNorthEast,1)) && (1 == size(stdNorthEast,2)) && isa(stdNorthEast, 'double'), 'GenericINS.UpdatePosition2D(): "stdNorthEast" must be a 2-by-1 vector of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.UpdatePosition2D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            obj.UpdatePosition(2, measurementLatLon, stdNorthEast, posBody2Sensor);
        end
        function UpdatePosition1D(obj, measurementAlt, stdDown, posBody2Sensor)
            %GenericINS.UpdatePosition1D Perform an observation step for 1D geographic position data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementAlt ... The measurement scalar [alt (m, positive upwards)].
            % stdDown        ... The standard deviation in meters, [down].
            % posBody2Sensor ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            assert(isscalar(measurementAlt) && isa(measurementAlt, 'double'), 'GenericINS.UpdatePosition1D(): "measurementAlt" must be a scalar of type double!');
            assert(isscalar(stdDown) && isa(stdDown, 'double'), 'GenericINS.UpdatePosition1D(): "stdDown" must be a scalar of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.UpdatePosition1D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            obj.UpdatePosition(1, measurementAlt, stdDown, posBody2Sensor);
        end
        function UpdateVelocity3D(obj, measurementXYZ, stdXYZ, posBody2Sensor, dcmBody2Sensor)
            %GenericINS.UpdateVelocity3D Perform an observation step for 3D velocity data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementXYZ ... The measurement vector, [x (m/s); y (m/s); z (m/s)].
            % stdXYZ         ... The standard deviation, [x (m/s); y (m/s); z (m/s)].
            % posBody2Sensor ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            % dcmBody2Sensor ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert((3 == size(measurementXYZ,1)) && (1 == size(measurementXYZ,2)) && isa(measurementXYZ, 'double'), 'GenericINS.UpdateVelocity3D(): "measurementXYZ" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdXYZ,1)) && (1 == size(stdXYZ,2)) && isa(stdXYZ, 'double'), 'GenericINS.UpdateVelocity3D(): "stdXYZ" must be a 3-by-1 vector of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.UpdateVelocity3D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.UpdateVelocity3D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateVelocity(3, measurementXYZ, stdXYZ, posBody2Sensor, dcmBody2Sensor);
        end
        function UpdateVelocity2D(obj, measurementXY, stdXY, posBody2Sensor, dcmBody2Sensor)
            %GenericINS.UpdateVelocity2D Perform an observation step for 2D velocity data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementXY  ... The measurement vector, [x (m/s); y (m/s)].
            % stdXY          ... The standard deviation, [x (m/s); y (m/s)].
            % posBody2Sensor ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            % dcmBody2Sensor ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert((2 == size(measurementXY,1)) && (1 == size(measurementXY,2)) && isa(measurementXY, 'double'), 'GenericINS.UpdateVelocity2D(): "measurementXY" must be a 2-by-1 vector of type double!');
            assert((2 == size(stdXY,1)) && (1 == size(stdXY,2)) && isa(stdXY, 'double'), 'GenericINS.UpdateVelocity2D(): "stdXY" must be a 2-by-1 vector of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.UpdateVelocity2D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.UpdateVelocity3D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateVelocity(2, measurementXY, stdXY, posBody2Sensor, dcmBody2Sensor);
        end
        function UpdateVelocity1D(obj, measurementZ, stdZ, posBody2Sensor, dcmBody2Sensor)
            %GenericINS.UpdateVelocity1D Perform an observation step for 1D velocity data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementZ   ... The measurement scalar, [z (m/s)].
            % stdZ           ... The standard deviation, [z (m/s)].
            % posBody2Sensor ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            % dcmBody2Sensor ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert(isscalar(measurementZ) && isa(measurementZ, 'double'), 'GenericINS.UpdateVelocity1D(): "measurementZ" must be a scalar of type double!');
            assert(isscalar(stdZ) && isa(stdZ, 'double'), 'GenericINS.UpdateVelocity1D(): "stdZ" must be a scalar of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.UpdateVelocity1D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.UpdateVelocity3D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateVelocity(1, measurementZ, stdZ, posBody2Sensor, dcmBody2Sensor);
        end
        function UpdateOrientation3D(obj, measurementRollPitchYaw, stdRollPitchYaw, dcmBody2Sensor)
            %GenericINS.UpdateOrientation3D Perform an observation step for 3D orientation data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementRollPitchYaw ... The measurement vector, [roll (rad); pitch (rad); yaw (rad)].
            % stdRollPitchYaw         ... The standard deviation, [roll (rad); pitch (rad); yaw (rad)].
            % dcmBody2Sensor          ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert((3 == size(measurementRollPitchYaw,1)) && (1 == size(measurementRollPitchYaw,2)) && isa(measurementRollPitchYaw, 'double'), 'GenericINS.UpdateOrientation3D(): "measurementRollPitchYaw" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdRollPitchYaw,1)) && (1 == size(stdRollPitchYaw,2)) && isa(stdRollPitchYaw, 'double'), 'GenericINS.UpdateOrientation3D(): "stdRollPitchYaw" must be a 3-by-1 vector of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.UpdateOrientation3D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateOrientation(3, measurementRollPitchYaw, stdRollPitchYaw, dcmBody2Sensor);
        end
        function UpdateOrientation2D(obj, measurementRollPitch, stdRollPitch, dcmBody2Sensor)
            %GenericINS.UpdateOrientation2D Perform an observation step for 2D orientation data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementRollPitch ... The measurement vector, [roll (rad); pitch (rad)].
            % stdRollPitch         ... The standard deviation, [roll (rad); pitch (rad)].
            % dcmBody2Sensor       ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert((2 == size(measurementRollPitch,1)) && (1 == size(measurementRollPitch,2)) && isa(measurementRollPitch, 'double'), 'GenericINS.UpdateOrientation2D(): "measurementRollPitch" must be a 2-by-1 vector of type double!');
            assert((2 == size(stdRollPitch,1)) && (1 == size(stdRollPitch,2)) && isa(stdRollPitch, 'double'), 'GenericINS.UpdateOrientation2D(): "stdRollPitch" must be a 2-by-1 vector of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.UpdateOrientation2D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateOrientation(2, measurementRollPitch, stdRollPitch, dcmBody2Sensor);
        end
        function UpdateOrientation1D(obj, measurementYaw, stdYaw, dcmBody2Sensor)
            %GenericINS.UpdateOrientation1D Perform an observation step for 1D orientation data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementYaw ... The measurement scalar, [yaw (rad)].
            % stdYaw         ... The standard deviation, [yaw (rad)].
            % dcmBody2Sensor ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert(isscalar(measurementYaw) && isa(measurementYaw, 'double'), 'GenericINS.UpdateOrientation1D(): "measurementYaw" must be a scalar of type double!');
            assert(isscalar(stdYaw) && isa(stdYaw, 'double'), 'GenericINS.UpdateOrientation1D(): "stdYaw" must be a scalar of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.UpdateOrientation1D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateOrientation(1, measurementYaw, stdYaw, dcmBody2Sensor);
        end
        function [valid, positionLLA, orientationQuaternionWXYZ, orientationRollPitchYaw, velocityNED, velocityUVW, velocityPQR, courseOverGround, speedOverGround, angleOfAttack, sideSlipAngle] = GetMotionState(obj)
            %GenericINS.GetMotionState Get the motion state from the context.
            % 
            % RETURN
            % valid                      ... A scalar boolean that indicates if the motion state is valid or not. It is valid if the INS was initialized and has performed at least one prediction.
            % positionLLA                ... Geographic position of the body frame, [lat (rad); lon (rad); alt (m, positive upwards)].
            % orientationQuaternionWXYZ  ... Unit quaternion describing a rotation from body frame to navigation frame, [qw; qx; qy; qz], where qw indicates the scalar part of the quaternion.
            % orientationRollPitchYaw    ... Euler angles according to the ZYX-convention.
            % velocityNED                ... Translational velocity [m/s] of the body frame given in navigation frame coordinates, excluding earth rotation and transport rate.
            % velocityUVW                ... Translational velocity [m/s] of the body frame given in body frame coordinates, excluding earth rotation and transport rate.
            % velocityPQR                ... Rotational velocity [rad/s] of the body frame with respect to the navigation frame, excluding earth rotation and transport rate.
            % courseOverGround           ... Course over ground angle in radians.
            % speedOverGround            ... Speed over ground in meters per second.
            % angleOfAttack              ... Angle of attack in radians.
            % sideSlipAngle              ... Side slip angle in radians.
            valid = obj.valid;

            % Orientation
            orientationQuaternionWXYZ = GenericINS.Normalize(obj.x(7:10));
            Cq = GenericINS.Cb2n(orientationQuaternionWXYZ);
            [roll, pitch, yaw] = GenericINS.EulerZYX(Cq);
            orientationRollPitchYaw = [roll; pitch; yaw];

            % Geographic position (add vector from accelerometer to body origin)
            iM = diag([(1/(obj.Rn + obj.x(3))); (1/((obj.Re + obj.x(3))*cos(obj.x(1)))); -1.0]);
            positionLLA = obj.x(1:3) - iM * Cq * obj.posIMUBody2Sensor;
            [lat, lon] = GenericINS.LatLon(positionLLA(1), positionLLA(2));
            positionLLA(1) = lat;
            positionLLA(2) = lon;

            % Rotational velocity (remove bias and align to body frame)
            w_ie = [obj.omegaEarth * cos(obj.x(1)); 0; -obj.omegaEarth * sin(obj.x(1))];
            w_en = [(obj.x(5) / ((obj.Re + obj.x(3)))); -(obj.x(4) / (obj.Rn + obj.x(3))); -(obj.x(5) / (obj.Re + obj.x(3)))*tan(obj.x(1))];
            velocityPQR = obj.dcmIMUBody2Sensor' * (obj.gyrRaw - obj.x(14:16)) - Cq' * (w_ie + w_en);

            % Velocity in the navigation frame
            velocityNED = obj.x(4:6) - Cq * cross(velocityPQR, obj.posIMUBody2Sensor);

            % Velocity in the body frame
            velocityUVW = Cq' * velocityNED;

            % COG, SOG, AOA, SSA
            courseOverGround = GenericINS.SymmetricalAngle(atan2(velocityNED(2), velocityNED(1)));
            speedOverGround = sqrt(velocityNED(1:2)'*velocityNED(1:2));
            angleOfAttack = GenericINS.SymmetricalAngle(atan2(velocityNED(3), speedOverGround));
            sideSlipAngle = GenericINS.SymmetricalAngle(courseOverGround - yaw);
        end
    end
    methods(Access=private)
        function GenerateSigmaPoints(obj)
            % Ensure normalized quaternion
            len = norm(obj.x(7:10));
            if(len < 100*eps)
                obj.x(7:10) = [1;0;0;0];
            else
                obj.x(7:10) = obj.x(7:10) ./ len;
            end

            % Matrix of sigma points (L x numSP) initialized with mean value x
            obj.Xi = repmat(obj.x, 1, GenericINS.NUM_SP);

            % For remaining sigma points S*Z is required
            SZ = obj.S * obj.Z(:,int32(2):GenericINS.NUM_SP);

            % Convert orientation-vector components (3 dimensions) to quaternion-components (4 dimensions): SZq will have one more row than SZ
            SZq = [SZ(1:6,:); zeros(4,GenericINS.NUM_SP - int32(1)); SZ(10:end,:)];
            SZq(7:10,:) = GenericINS.OV2Q(SZ(7:9,:));

            % Calculate final sigma points by Xi = x + S*Z, however, with special treatment for quaternion parts (quaternion multiplication instead of vector addition)
            % We leave Xi(:,1) as it is because the 0-th sigma point is equal to the mean value
            obj.Xi(1:6,2:end)    = obj.Xi(1:6,2:end) + SZq(1:6,:);
            obj.Xi(7:10,2:end)   = GenericINS.Qdot(SZq(7:10,:), obj.Xi(7:10,1)); 
            obj.Xi(11:end,2:end) = obj.Xi(11:end,2:end) + SZq(11:end,:);
        end
        function CalculateDX(obj)
            % Get the pivot angle from 0-th sigma point and remove this angle from all sigma points
            pivotAngle = obj.Xi(2,1);
            obj.Xi(2,:) = GenericINS.SymmetricalAngle(obj.Xi(2,:) - repmat(pivotAngle, 1, GenericINS.NUM_SP));

            % Calculate weighted mean of sigma points (barycentric mean for quaternion part)
            obj.x(1:GenericINS.DIM_X) = obj.w0 * obj.Xi(1:GenericINS.DIM_X,1) + obj.wi * sum(obj.Xi(1:GenericINS.DIM_X,2:GenericINS.NUM_SP), 2);
            obj.x(7:10) = GenericINS.Normalize(obj.x(7:10));

            % Calculate dX = Xi - x
            % Special treatment for angle and quaternion. Xi - x: We need the inverse quaternion because of -x.
            q_inv = [obj.x(7); -obj.x(8:10)];
            dXq = GenericINS.Qdot(obj.Xi(7:10,:), q_inv);
            obj.dX = [obj.Xi(1,:) - repmat(obj.x(1), 1, GenericINS.NUM_SP); ...
                  GenericINS.SymmetricalAngle(obj.Xi(2,:) - repmat(obj.x(2), 1, GenericINS.NUM_SP)); ...
                  obj.Xi(3:6,:) - repmat(obj.x(3:6), 1, GenericINS.NUM_SP); ...
                  GenericINS.Q2OV(dXq); ...
                  obj.Xi(11:GenericINS.DIM_X,:) - repmat(obj.x(11:GenericINS.DIM_X), 1, GenericINS.NUM_SP)];

            % Add pivot angle to state (and also sigma-points for an upcomming measurement update)
            obj.x(2) = GenericINS.SymmetricalAngle(obj.x(2) + pivotAngle);
            obj.Xi(2,:) = GenericINS.SymmetricalAngle(obj.Xi(2,:) + repmat(pivotAngle, 1, GenericINS.NUM_SP));
        end
        function UpdatePosition(obj, yDim, measurement, stdMeasurement, posBody2Sensor)
            % Do not allow updates of an uninitialized INS
            if(~obj.initialized)
                return;
            end

            % If sigma-points are not up-to-date: generate a new set of sigma-points, else use existing set.
            if(~obj.spU2D)
                obj.GenerateSigmaPoints();
                obj.CalculateDX();
            end

            % Sigma-points are no longer up-to-date for next Update()-call
            obj.spU2D = false;

            % Propagate through measurement model
            yDim = int32(yDim);
            Y = zeros(yDim, GenericINS.NUM_SP);
            switch(yDim)
                case 1
                    for i = int32(1):GenericINS.NUM_SP
                        state = obj.Xi(1:GenericINS.DIM_X,i);
                        y = GenericINS.SensorModelPosition(state, obj.Rn, obj.Re, obj.posIMUBody2Sensor, posBody2Sensor);
                        Y(:,i) = y(3);
                    end
                case 2
                    for i = int32(1):GenericINS.NUM_SP
                        state = obj.Xi(1:GenericINS.DIM_X,i);
                        y = GenericINS.SensorModelPosition(state, obj.Rn, obj.Re, obj.posIMUBody2Sensor, posBody2Sensor);
                        Y(:,i) = y(1:2);
                    end
                case 3
                    for i = int32(1):GenericINS.NUM_SP
                        state = obj.Xi(1:GenericINS.DIM_X,i);
                        Y(:,i) = GenericINS.SensorModelPosition(state, obj.Rn, obj.Re, obj.posIMUBody2Sensor, posBody2Sensor);
                    end
            end

            % Get pivot angle from 0-th Y sigma-point and remove this angle from all sigma-points
            pivotAngle = 0;
            if(yDim > int32(1))
                pivotAngle = Y(2,1);
                Y(2,:) = GenericINS.SymmetricalAngle(Y(2,:) - repmat(pivotAngle, 1, GenericINS.NUM_SP));
            end

            % Calculate weighted mean of sigma-points
            ym = obj.w0 * Y(:,1) + obj.wi * sum(Y(:,int32(2):GenericINS.NUM_SP), 2);

            % Calculate dY = Y - ym
            dY = (Y - repmat(ym, 1, GenericINS.NUM_SP));

            % Add pivot angle to mean value
            if(yDim > int32(1))
                ym(2) = GenericINS.SymmetricalAngle(ym(2) + pivotAngle);
            end

            % Calculate the innovation
            innovation = measurement - ym;
            if(yDim > int32(1))
                innovation(2) = GenericINS.SymmetricalAngle(innovation(2));
            end

            % Calculate uncertainty: [m]->[rad] for lat,lon
            sqrtR = diag(stdMeasurement);
            if(yDim > int32(1))
                sqrtR(1,1) = sqrtR(1,1) / (obj.Rn + obj.x(3));
                sqrtR(2,2) = sqrtR(2,2) / ((obj.Re + obj.x(3)) * cos(obj.x(1)));
            end

            % QR decomposition of sigma points: additive SRSSUKF
            [~, Sy_UPPER] = qr([(obj.srwi * dY(:,2:GenericINS.NUM_SP)) sqrtR]', 0);

            % Cholesky update or downdate (w0 is guaranteed to be positive -> update)
            Sy_UPPER = cholupdate(Sy_UPPER, obj.srw0 * dY(:,1), '+');
            Sy = Sy_UPPER';

            % Calculate covariance matrix Pxy
            Pxy = obj.w0 * obj.dX(:,1) * dY(:,1)' + obj.wi * obj.dX(:,int32(2):GenericINS.NUM_SP) * dY(:,int32(2):GenericINS.NUM_SP)';

            % Calculate the Kalman gain matrix
            K = (Pxy / Sy') / Sy;

            % Get state update dx = K * innovation
            dx = K * innovation;

            % Update state estimation x = x + dx taking angles and quaternions into account
            obj.x(1:6) = obj.x(1:6) + dx(1:6);
            obj.x(2) = GenericINS.SymmetricalAngle(obj.x(2));
            obj.x(7:10) = GenericINS.Qdot(GenericINS.OV2Q(dx(7:9)), obj.x(7:10));
            obj.x(11:GenericINS.DIM_X) = obj.x(11:GenericINS.DIM_X) + dx(10:end);

            % Update sqrt of covariance using cholesky downdate. Because MATLABs cholesky update uses the UPPER triangle we have to transpose Sx.
            Sx_UPPER = obj.S(int32(1):GenericINS.DIM_XS,int32(1):GenericINS.DIM_XS)';
            U = K * Sy;
            for j = 1:yDim
                [Sx_UPPER,~] = cholupdate(Sx_UPPER, U(:,j), '-');
            end

            % Convert from UPPER cholesky factor to LOWER cholesky factor
            obj.S(int32(1):GenericINS.DIM_XS,int32(1):GenericINS.DIM_XS) = Sx_UPPER';
        end
        function UpdateVelocity(obj, yDim, measurement, stdMeasurement, posBody2Sensor, dcmBody2Sensor)
            % Do not allow updates of an uninitialized INS
            if(~obj.initialized)
                return;
            end

            % If sigma-points are not up-to-date: generate a new set of sigma-points, else use existing set.
            if(~obj.spU2D)
                obj.GenerateSigmaPoints();
                obj.CalculateDX();
            end

            % Sigma-points are no longer up-to-date for next Update()-call
            obj.spU2D = false;

            % Propagate through measurement model
            yDim = int32(yDim);
            Y = zeros(yDim, GenericINS.NUM_SP);
            switch(yDim)
                case 1
                    for i = int32(1):GenericINS.NUM_SP
                        state = obj.Xi(1:GenericINS.DIM_X,i);
                        y = GenericINS.SensorModelVelocity(state, obj.omegaEarth, obj.gyrRaw, obj.posIMUBody2Sensor, obj.dcmIMUBody2Sensor, posBody2Sensor, dcmBody2Sensor);
                        Y(:,i) = y(3);
                    end
                case 2
                    for i = int32(1):GenericINS.NUM_SP
                        state = obj.Xi(1:GenericINS.DIM_X,i);
                        y = GenericINS.SensorModelVelocity(state, obj.omegaEarth, obj.gyrRaw, obj.posIMUBody2Sensor, obj.dcmIMUBody2Sensor, posBody2Sensor, dcmBody2Sensor);
                        Y(:,i) = y(1:2);
                    end
                case 3
                    for i = int32(1):GenericINS.NUM_SP
                        state = obj.Xi(1:GenericINS.DIM_X,i);
                        Y(:,i) = GenericINS.SensorModelVelocity(state, obj.omegaEarth, obj.gyrRaw, obj.posIMUBody2Sensor, obj.dcmIMUBody2Sensor, posBody2Sensor, dcmBody2Sensor);
                    end
            end

            % Calculate weighted mean of sigma-points
            ym = obj.w0 * Y(:,1) + obj.wi * sum(Y(:,int32(2):GenericINS.NUM_SP), 2);

            % Calculate dY = Y - ym
            dY = (Y - repmat(ym, 1, GenericINS.NUM_SP));

            % Calculate the innovation
            innovation = measurement - ym;

            % QR decomposition of sigma points: additive SRSSUKF
            sqrtR = diag(stdMeasurement);
            [~, Sy_UPPER] = qr([(obj.srwi * dY(:,2:GenericINS.NUM_SP)) sqrtR]', 0);

            % Cholesky update or downdate (w0 is guaranteed to be positive -> update)
            Sy_UPPER = cholupdate(Sy_UPPER, obj.srw0 * dY(:,1), '+');
            Sy = Sy_UPPER';

            % Calculate covariance matrix Pxy
            Pxy = obj.w0 * obj.dX(:,1) * dY(:,1)' + obj.wi * obj.dX(:,int32(2):GenericINS.NUM_SP) * dY(:,int32(2):GenericINS.NUM_SP)';

            % Calculate the Kalman gain matrix
            K = (Pxy / Sy') / Sy;

            % Get state update dx = K * innovation
            dx = K * innovation;

            % Update state estimation x = x + dx taking angles and quaternions into account
            obj.x(1:6) = obj.x(1:6) + dx(1:6);
            obj.x(2) = GenericINS.SymmetricalAngle(obj.x(2));
            obj.x(7:10) = GenericINS.Qdot(GenericINS.OV2Q(dx(7:9)), obj.x(7:10));
            obj.x(11:GenericINS.DIM_X) = obj.x(11:GenericINS.DIM_X) + dx(10:end);

            % Update sqrt of covariance using cholesky downdate. Because MATLABs cholesky update uses the UPPER triangle we have to transpose Sx.
            Sx_UPPER = obj.S(int32(1):GenericINS.DIM_XS,int32(1):GenericINS.DIM_XS)';
            U = K * Sy;
            for j = 1:yDim
                [Sx_UPPER,~] = cholupdate(Sx_UPPER, U(:,j), '-');
            end

            % Convert from UPPER cholesky factor to LOWER cholesky factor
            obj.S(int32(1):GenericINS.DIM_XS,int32(1):GenericINS.DIM_XS) = Sx_UPPER';
        end
        function UpdateOrientation(obj, yDim, measurement, stdMeasurement, dcmBody2Sensor)
            % Do not allow updates of an uninitialized INS
            if(~obj.initialized)
                return;
            end

            % If sigma-points are not up-to-date: generate a new set of sigma-points, else use existing set.
            if(~obj.spU2D)
                obj.GenerateSigmaPoints();
                obj.CalculateDX();
            end

            % Sigma-points are no longer up-to-date for next Update()-call
            obj.spU2D = false;

            % Propagate through measurement model
            yDim = int32(yDim);
            Y = zeros(yDim, GenericINS.NUM_SP);
            switch(yDim)
                case 1
                    for i = int32(1):GenericINS.NUM_SP
                        state = obj.Xi(1:GenericINS.DIM_X,i);
                        y = GenericINS.SensorModelOrientation(state, dcmBody2Sensor);
                        Y(:,i) = y(3);
                    end
                case 2
                    for i = int32(1):GenericINS.NUM_SP
                        state = obj.Xi(1:GenericINS.DIM_X,i);
                        y = GenericINS.SensorModelOrientation(state, dcmBody2Sensor);
                        Y(:,i) = y(1:2);
                    end
                case 3
                    for i = int32(1):GenericINS.NUM_SP
                        state = obj.Xi(1:GenericINS.DIM_X,i);
                        Y(:,i) = GenericINS.SensorModelOrientation(state, dcmBody2Sensor);
                    end
            end

            % Get pivot angle from 0-th Y sigma-point and remove this angle from all sigma-points
            pivotAngle = Y(:,1);
            Y = GenericINS.SymmetricalAngle(Y - repmat(pivotAngle, 1, GenericINS.NUM_SP));

            % Calculate weighted mean of sigma-points
            ym = obj.w0 * Y(:,1) + obj.wi * sum(Y(:,int32(2):GenericINS.NUM_SP), 2);

            % Calculate dY = Y - ym
            dY = (Y - repmat(ym, 1, GenericINS.NUM_SP));

            % Add pivot angle to mean value
            ym = GenericINS.SymmetricalAngle(ym + pivotAngle);

            % Calculate the innovation
            innovation = GenericINS.SymmetricalAngle(measurement - ym);

            % QR decomposition of sigma points: additive SRSSUKF
            sqrtR = diag(stdMeasurement);
            [~, Sy_UPPER] = qr([(obj.srwi * dY(:,2:GenericINS.NUM_SP)) sqrtR]', 0);

            % Cholesky update or downdate (w0 is guaranteed to be positive -> update)
            Sy_UPPER = cholupdate(Sy_UPPER, obj.srw0 * dY(:,1), '+');
            Sy = Sy_UPPER';

            % Calculate covariance matrix Pxy
            Pxy = obj.w0 * obj.dX(:,1) * dY(:,1)' + obj.wi * obj.dX(:,int32(2):GenericINS.NUM_SP) * dY(:,int32(2):GenericINS.NUM_SP)';

            % Calculate the Kalman gain matrix
            K = (Pxy / Sy') / Sy;

            % Get state update dx = K * innovation
            dx = K * innovation;

            % Update state estimation x = x + dx taking angles and quaternions into account
            obj.x(1:6) = obj.x(1:6) + dx(1:6);
            obj.x(2) = GenericINS.SymmetricalAngle(obj.x(2));
            obj.x(7:10) = GenericINS.Qdot(GenericINS.OV2Q(dx(7:9)), obj.x(7:10));
            obj.x(11:GenericINS.DIM_X) = obj.x(11:GenericINS.DIM_X) + dx(10:end);

            % Update sqrt of covariance using cholesky downdate. Because MATLABs cholesky update uses the UPPER triangle we have to transpose Sx.
            Sx_UPPER = obj.S(int32(1):GenericINS.DIM_XS,int32(1):GenericINS.DIM_XS)';
            U = K * Sy;
            for j = 1:yDim
                [Sx_UPPER,~] = cholupdate(Sx_UPPER, U(:,j), '-');
            end

            % Convert from UPPER cholesky factor to LOWER cholesky factor
            obj.S(int32(1):GenericINS.DIM_XS,int32(1):GenericINS.DIM_XS) = Sx_UPPER';
        end
    end
    methods(Static,Access=private)
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
        function y = SymmetricalAngle(x)
            %GenericINS.SymmetricalAngle Convert a given angle x [rad] to an output angle y [rad] with y being in range [-pi, +pi).
            % 
            % PARAMETERS
            % x ... Input angle in radians, either scalar or n-dimensional.
            % 
            % RETURN
            % y ... Output angle in radians being in range [-pi, pi).
            x = double(mod(x, pi+pi));
            y = x - double(x >= pi) * (pi+pi);
        end
        function Q = OV2Q(OV)
            %GenericINS.OV2Q Convert an orientation vector to a unit quaternion.
            % 
            % PARAMETERS
            % ov ... 3xN matrix containing orientation vectors.
            % 
            % RETURN
            % q ... 4xN corresponding unit quaternion matrix.
            assert(3 == size(OV,1));
            N = size(OV,2);
            Q = zeros(4, N);
            for n=1:N
                len = norm(OV(:,n));
                Q(1,n) = 1.0;
                if(len > eps)
                    a = 0.5 * len;
                    s = sin(a) / len;
                    Q(1,n) = cos(a);
                    Q(2,n) = OV(1,n) * s;
                    Q(3,n) = OV(2,n) * s;
                    Q(4,n) = OV(3,n) * s;
                end
            end
        end
        function OV = Q2OV(Q)
            %GenericINS.Q2OV Convert a unit quaternion to an orientation vector. The input quaternion will be normalized.
            % 
            % PARAMETERS
            % q ... 4xN matrix of unit quaternions, will be normalized.
            % 
            % RETURN
            % ov ... 3xN matrix of corresponding orientation vectors.
            assert(4 == size(Q,1));
            N = size(Q,2);
            OV = zeros(3,N);
            for n=1:N
                qn = Q(:,n) / norm(Q(:,n));
                angle_half = acos(qn(1));
                s = sin(angle_half);
                if(abs(s) > 100*eps)
                    s_inv = 1.0 / s;
                    OV(1,n) = qn(2) * s_inv;
                    OV(2,n) = qn(3) * s_inv;
                    OV(3,n) = qn(4) * s_inv;
                    OV(:,n) = 2.0*angle_half * OV(:,n);
                end
            end
        end
        function Y = Qdot(Q, r)
            %GenericINS.Qdot Calculate the quaternion multiplication for two unit quaternions Q and r. The output quaternion will be a unit quaternion.
            % 
            % PARAMETERS
            % Q ... 4xN Matrix of normalized unit quaternions (left hand side).
            % r ... 4x1 normalized unit quaternion (right hand side).
            % 
            % RETURN
            % Y ... 4xN unit quaternions.
            assert(4 == size(Q,1));
            assert(4 == size(r,1));
            assert(1 == size(r,2));
            N = size(Q,2);
            Y = zeros(4, N);
            for n = 1:N
                mat = [Q(1,n) -Q(2,n) -Q(3,n) -Q(4,n); Q(2,n) Q(1,n) -Q(4,n) Q(3,n); Q(3,n) Q(4,n) Q(1,n) -Q(2,n); Q(4,n) -Q(3,n) Q(2,n) Q(1,n)];
                x = mat * r;
                Y(:,n) = x / norm(x);
            end
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
            roll = GenericINS.SymmetricalAngle(atan2(R(3,2), R(3,3)));
            pitch = -real(asin(min(max(R(3,1),-1.0),1.0)));
            yaw = GenericINS.SymmetricalAngle(atan2(R(2,1), R(1,1)));
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
        function g = Gravity(altitude, latitude, R0)
            %GenericINS.Gravity Compute the local gravity g [m/s^2] for a given position using the WGS84 earth reference model.
            % 
            % PARAMETERS
            % altitude ... Height [m] w.r.t. the WGS84 reference ellipsoid (positive upwards).
            % latitude ... Geographical latitude [rad] for the position of interest.
            % R0       ... Mean earth radius [m] (WGS84).
            % 
            % RETURN
            % g ... Scalar local gravity [m/s^2].
            s = sin(latitude);
            s2 = sin(2 * latitude);
            R = R0 / (R0 + altitude);
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
        function xOut = ProcessModelEuler(x, w, u, Ts, dcmIMUBody2Sensor, Rn, Re, omegaEarth, localGravity)
            % Output size
            xOut = x;

            % Inputs: remove estimated biases and align to b-frame
            f_ib = dcmIMUBody2Sensor' * (u(1:3) + w(1:3) - x(11:13));
            w_ib = dcmIMUBody2Sensor' * (u(4:6) + w(4:6) - x(14:16));

            % Ensure Current attitude
            x(7:10) = GenericINS.Normalize(x(7:10));
            DCM_b2n = GenericINS.Cb2n(x(7:10));
            DCM_n2b = DCM_b2n';

            % Predict position
            xOut(1) = x(1) + Ts * (x(4) / (Rn + x(3)));
            xOut(2) = x(2) + Ts * (x(5) / ((Re + x(3)) * cos(x(1))));
            xOut(3) = x(3) - Ts * (x(6));

            % lat/lon range conversion
            [lat, lon] = GenericINS.LatLon(xOut(1), xOut(2));
            xOut(1) = lat;
            xOut(2) = lon;

            % Predict velocity
            w_ie = [omegaEarth * cos(x(1)); 0; -omegaEarth * sin(x(1))];
            w_en = [(x(5) / ((Re + x(3)))); -(x(4) / (Rn + x(3))); -(x(5) / (Re + x(3)))*tan(x(1))];
            coriolis = cross(2 * w_ie + w_en, x(4:6));
            xOut(4:6) = x(4:6) + Ts * (DCM_b2n * f_ib - coriolis + [0.0; 0.0; localGravity]);

            % Predict attitude
            w_nb = w_ib - DCM_n2b * (w_ie + w_en);
            Omega = [0 -w_nb(1) -w_nb(2) -w_nb(3); w_nb(1) 0 w_nb(3) -w_nb(2); w_nb(2) -w_nb(3) 0 w_nb(1); w_nb(3) w_nb(2) -w_nb(1) 0];
            xOut(7:10) = (eye(4)*(1 - Ts*Ts*(w_nb(1)*w_nb(1) + w_nb(2)*w_nb(2) + w_nb(3)*w_nb(3))/8) + 0.5*Ts*Omega) * x(7:10);
            xOut(7:10) = GenericINS.Normalize(xOut(7:10));

            % Predict inertial biases (Random Walk)
            xOut(11:13) = x(11:13) + w(7:9);
            xOut(14:16) = x(14:16) + w(10:12);
        end
        function y = SensorModelPosition(x, Rn, Re, posIMUBody2Sensor, posPOSBody2Sensor)
            % Obtain position and attitude from state x
            phi = x(1);
            lambda = x(2);
            alt = x(3);
            q = GenericINS.Normalize(x(7:10));

            % The position of the sensor
            M = diag([(1/(Rn + alt)); (1/((Re + alt) * cos(phi))); -1.0]);
            p = [phi; lambda; alt] + M * GenericINS.Cb2n(q) * (posPOSBody2Sensor - posIMUBody2Sensor);

            % lat/lon range conversion
            [lat, lon] = GenericINS.LatLon(p(1), p(2));
            y = [lat; lon; p(3)];
        end
        function y = SensorModelVelocity(x, omegaEarth, gyrRaw, posIMUBody2Sensor, dcmIMUBody2Sensor, posBody2Sensor, dcmBody2Sensor)
            % Direction cosine matrices from current quaternion
            DCM_b2n = GenericINS.Cb2n(GenericINS.Normalize(x(7:10)));
            DCM_n2b = DCM_b2n';

            % Angular rate of body with respect to the earth
            w_ie = omegaEarth * [cos(x(1)); 0; -sin(x(1))];
            w_eb = dcmIMUBody2Sensor' * (gyrRaw - x(14:16)) - DCM_n2b * w_ie;

            % Additional velocity due to angular speed + sensor alignment in b-frame
            v_b = cross(w_eb, posBody2Sensor - posIMUBody2Sensor);

            % Transform additional b-frame velocity to n-frame and add current prediction of v_eb (n-frame)
            v_n = x(4:6) + DCM_b2n * v_b;

            % Finally, transform velocity to sensor frame of the DVL
            y = dcmBody2Sensor * DCM_n2b * v_n;
        end
        function y = SensorModelOrientation(x, dcmBody2Sensor)
            % Get attitude from state vector
            q = GenericINS.Normalize(x(7:10));

            % Transform to roll, pitch, yaw
            DCM_S2N = GenericINS.Cb2n(q) * dcmBody2Sensor';
            [roll,pitch,yaw] = GenericINS.EulerZYX(DCM_S2N);

            % Output
            y = [roll; pitch; yaw];
        end
    end
    properties(Constant,Access=private)
        % Fixed dimension for generic INS problem: DO NOT CHANGE!
        DIM_X  = int32(16);                                   % Dimension of state vector (x).
        DIM_XS = GenericINS.DIM_X - int32(1);                 % Dimension of state vector (x) with respect to unvertainty S.
        DIM_W  = int32(12);                                   % Dimension of process noise (w).
        DIM_L  = GenericINS.DIM_X + GenericINS.DIM_W;         % Dimension of augmented state vector.
        DIM_LS = GenericINS.DIM_XS + GenericINS.DIM_W;        % Dimension of augmented state vector with respect to uncertainty S.
        NUM_SP = GenericINS.DIM_LS + int32(2);                % Number of sigma-points.
    end
    properties(Access=private)
        initialized;       % True if filter is initialized, false otherwise.
        valid;             % True if filter is initialized and output is valid (at least one prediction), false otherwise.
        Z;                 % Spherical simplex sigma point matrix.
        x;                 % Augmented state vector (L x 1).
        S;                 % Augmented square-root covariance matrix.
        Xi;                % Matrix of sigma points (L x numSP).
        dX;                % Xi - x (DIM_XS x numSP).
        spU2D;             % True if sigma points are up to date, false otherwise.
        gyrRaw;            % Raw gyroscope data from IMU (sensor frame).
        dcmIMUBody2Sensor; % Direction cosine matrix for rotation from body frame to sensor frame.
        posIMUBody2Sensor; % Position of IMU sensor frame in body frame (b-frame origin to s-frame origin in b-frame coords).
        omegaEarth;        % Angular rate [rad/s] of the earth w.r.t. the inertial frame calculated during prediction.
        Rn;                % Northern earth radius [m] calculated during prediction.
        Re;                % Eastern earth radius [m] calculated during prediction.
        localGravity;      % Scalar local gravity [m/s^2] calculated during prediction.
        w0;                % Scalar weighting factor for 0-th sigma point, range: [0;1).
        wi;                % Weighting factor for other sigma points (calculated from w0)
        srw0;              % Square root of w0.
        srwi;              % Square root of wi.
    end
end


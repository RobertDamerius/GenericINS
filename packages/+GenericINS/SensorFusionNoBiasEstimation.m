% SensorFusionNoBiasEstimation.m
% 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Version     Author                 Changes
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 20220307    Robert Damerius        Initial release.
% 20220309    Robert Damerius        Updated the gravity model based on MATLAB's gravitywgs84 implementation.
% 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
classdef SensorFusionNoBiasEstimation < handle
    methods(Static)
        function code = GenerateFunction(functionName, positionMeasurements, velocityMeasurements, orientationMeasurements, numSOGMeasurements)
            %GenericINS.SensorFusionNoBiasEstimation.GenerateFunction Generate a MATLAB function that implements a complete INS algorithm for a specific sensor configuration.
            % 
            % PARAMETERS
            % functionName            ... The name of the function. The function will be saved as [functionName '.m'].
            % positionMeasurements    ... N-by-1 vector with N being the number of position measurement sensors. Each element in the vector indicates the dimension. Possible values are:
            %                               1: Position sensor that measures only altitude.
            %                               2: Position sensor that measures only latitude and longitude.
            %                               3: Position sensor that measures latitude, longitude and altitude.
            % velocityMeasurements    ... N-by-1 vector with N being the number of velocity measurement sensors. Each element in the vector indicates the dimension. Possible values are:
            %                               1: Velocity sensor that measures only in z-direction.
            %                               2: Velocity sensor that measures only in x- and y-direction.
            %                               3: Velocity sensor that measures in all three directions.
            % orientationMeasurements ... N-by-1 vector with N being the number of orientation measurement sensors. Each element in the vector indicates the dimension. Possible values are:
            %                               1: Orientation sensor that measures only yaw.
            %                               2: Orientation sensor that measures only roll and pitch.
            %                               3: Orientation sensor that measures roll, pitch and yaw.
            % numSOGMeasurements      ... Scalar value representing the number of SOG measurements.
            % 
            % RETURN
            % code ... The generated code as a string.

            % Check for correct inputs
            positionMeasurements = int32(positionMeasurements);
            velocityMeasurements = int32(velocityMeasurements);
            orientationMeasurements = int32(orientationMeasurements);
            numSOGMeasurements = int32(numSOGMeasurements);
            numPositionMeasurements = int32(length(positionMeasurements));
            numVelocityMeasurements = int32(length(velocityMeasurements));
            numOrientationMeasurements = int32(length(orientationMeasurements));
            positionMeasurements = reshape(positionMeasurements, int32(1), numPositionMeasurements);
            velocityMeasurements = reshape(velocityMeasurements, int32(1), numVelocityMeasurements);
            orientationMeasurements = reshape(orientationMeasurements, int32(1), numOrientationMeasurements);
            assert(0 == (sum(positionMeasurements < 1) + sum(positionMeasurements > 3)), 'GenericINS.SensorFusionNoBiasEstimation.GenerateFunction(): "positionMeasurements" must only contain integer values {1, 2, 3}!');
            assert(0 == (sum(velocityMeasurements < 1) + sum(velocityMeasurements > 3)), 'GenericINS.SensorFusionNoBiasEstimation.GenerateFunction(): "velocityMeasurements" must only contain integer values {1, 2, 3}!');
            assert(0 == (sum(orientationMeasurements < 1) + sum(orientationMeasurements > 3)), 'GenericINS.SensorFusionNoBiasEstimation.GenerateFunction(): "orientationMeasurements" must only contain integer values {1, 2, 3}!');
            assert(isscalar(numSOGMeasurements), 'GenericINS.SensorFusionNoBiasEstimation.GenerateFunction(): "numSOGMeasurements" must be scalar!');


            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            % Generate argument list, documentation for function and variable code sections
            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            varCodeSectionA = '';
            varCodeSectionB = '';

            % Default arguments
            docs = ['    %%' functionName ' Generated MATLAB function for a generic inertial navigation system (' num2str(numPositionMeasurements) ' position measurements, ' num2str(numVelocityMeasurements) ' velocity measurements, ' num2str(numOrientationMeasurements) ' orientation measurements).\n'];
            docs = [docs '    %% This function was automatically generated using the command GenericINS.SensorFusionNoBiasEstimation.GenerateFunction(''' functionName ''', [' num2str(positionMeasurements) '], [' num2str(velocityMeasurements) '], [' num2str(orientationMeasurements) '], ' num2str(numSOGMeasurements) ').\n'];
            docs = [docs '    %% \n'];
            docs = [docs '    %% PARAMETERS\n'];
            docs = [docs '    %% time                       ... A scalar value indicating a monotonically increasing time in seconds.\n'];
            docs = [docs '    %% initialState               ... 9-by-1 initial state vector to be used when the INS is reset. The elements are as follows:\n'];
            docs = [docs '    %%                                 1: latitude     [rad]     Initial latitude of the position of the IMU.\n'];
            docs = [docs '    %%                                 2: longitude    [rad]     Initial longitude of the position of the IMU.\n'];
            docs = [docs '    %%                                 3: altitude     [m]       Initial altitude of the position of the IMU, positive upwards.\n'];
            docs = [docs '    %%                                 4: velocityU    [m/s]     Initial body-fixed velocity in x-direction of the body-frame at the position of the IMU.\n'];
            docs = [docs '    %%                                 5: velocityV    [m/s]     Initial body-fixed velocity in y-direction of the body-frame at the position of the IMU.\n'];
            docs = [docs '    %%                                 6: velocityW    [m/s]     Initial body-fixed velocity in z-direction of the body-frame at the position of the IMU.\n'];
            docs = [docs '    %%                                 7: roll         [rad]     Initial roll angle.\n'];
            docs = [docs '    %%                                 8: pitch        [rad]     Initial pitch angle.\n'];
            docs = [docs '    %%                                 9: yaw          [rad]     Initial yaw angle.\n'];
            docs = [docs '    %% initialStdDev              ... 9-by-1 standard deviation vector that represents the unvertainty of the initial state. It is better to use larger values than\n'];
            docs = [docs '    %%                                too small values for the filter to converge. The elements are as follows:\n'];
            docs = [docs '    %%                                 1: latitude     [rad]     Initial standard deviation for initial latitude.\n'];
            docs = [docs '    %%                                 2: longitude    [rad]     Initial standard deviation for initial longitude.\n'];
            docs = [docs '    %%                                 3: altitude     [m]       Initial standard deviation for initial altitude.\n'];
            docs = [docs '    %%                                 4: velocityU    [m/s]     Initial standard deviation for initial body-fixed velocity in x-direction of the body-frame.\n'];
            docs = [docs '    %%                                 5: velocityV    [m/s]     Initial standard deviation for initial body-fixed velocity in y-direction of the body-frame.\n'];
            docs = [docs '    %%                                 6: velocityW    [m/s]     Initial standard deviation for initial body-fixed velocity in z-direction of the body-frame.\n'];
            docs = [docs '    %%                                 7: orientationX           Initial standard deviation for x-component of orientation vector (axis-angle representation).\n'];
            docs = [docs '    %%                                 8: orientationY           Initial standard deviation for y-component of orientation vector (axis-angle representation).\n'];
            docs = [docs '    %%                                 9: orientationZ           Initial standard deviation for z-component of orientation vector (axis-angle representation).\n'];
            docs = [docs '    %% reset                      ... Greater than 0.0 if filter should be reset/reinitialized, 0.0 otherwise. During an initialization the initial state and initial standard deviations are used.\n'];
            docs = [docs '    %% sampletime                 ... Discrete sampletime in seconds to be used for forward euler integration. Should be the elapsed time to the latest prediction.\n'];
            docs = [docs '    %% IMU_Data                   ... The 7-by-1 vector representing the IMU data where the elements are as follows:\n'];
            docs = [docs '    %%                                 1: trigger                Greater than 0.0 if new IMU data is available and should be used in a prediction step, 0.0 otherwise.\n'];
            docs = [docs '    %%                                 2: accX         [m/(s*s)] Acceleration in x-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                                 3: accY         [m/(s*s)] Acceleration in y-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                                 4: accZ         [m/(s*s)] Acceleration in z-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                                 5: gyrX         [rad/s]   Angular rate around x-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                                 6: gyrY         [rad/s]   Angular rate around y-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                                 7: gyrZ         [rad/s]   Angular rate around z-axis of IMU sensor frame.\n'];
            docs = [docs '    %% IMU_StdDev                 ... 6-by-1 vector representing the standard deviation for inertial measurements and inertial biases. The elements are as follows:\n'];
            docs = [docs '    %%                                 1: accX         [m/(s*s)] Standard deviation for acceleration in x-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                                 2: accY         [m/(s*s)] Standard deviation for acceleration in y-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                                 3: accZ         [m/(s*s)] Standard deviation for acceleration in z-direction of IMU sensor frame.\n'];
            docs = [docs '    %%                                 4: gyrX         [rad/s]   Standard deviation for angular rate around x-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                                 5: gyrY         [rad/s]   Standard deviation for angular rate around y-axis of IMU sensor frame.\n'];
            docs = [docs '    %%                                 6: gyrZ         [rad/s]   Standard deviation for angular rate around z-axis of IMU sensor frame.\n'];
            docs = [docs '    %% IMU_PositionBody2Sensor    ... Position of IMU sensor frame origin w.r.t. body frame origin in body frame coordinates (meters).\n'];
            docs = [docs '    %% IMU_DCMBody2Sensor         ... Direction cosine matrix to rotate measurements from body frame to sensor frame.\n'];
            args = 'initialState, initialStdDev, reset, sampletime, IMU_Data, IMU_StdDev, IMU_PositionBody2Sensor, IMU_DCMBody2Sensor';

            % Position measurements: additional arguments, documentation and variable code sections
            for n = int32(1):numPositionMeasurements
                dim = positionMeasurements(n);
                arg_Pn_Data = ['P' num2str(n) '_Data'];
                arg_Pn_StdDev = ['P' num2str(n) '_StdDev'];
                arg_Pn_PositionBody2Sensor = ['P' num2str(n) '_PositionBody2Sensor'];
                arg_Pn_Buffer = ['P' num2str(n) '_Buffer'];
                args = [args ', ' arg_Pn_Data ', ' arg_Pn_StdDev ', ' arg_Pn_PositionBody2Sensor];
                docs = [docs '    %% ' arg_Pn_Data '                    ... ' num2str(int32(1) + dim) '-by-1 vector representing the position measurements of position sensor ' num2str(n) '. The elements are as follows:\n'];
                docs = [docs '    %%                                 1: trigger                Greater than 0.0 if new measurement data is available and should be used in an update step, 0.0 otherwise.\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                                 2: altitude     [m]       The altitude measurement.\n'];
                    case 2
                        docs = [docs '    %%                                 2: latitude     [rad]     The latitude measurement.\n'];
                        docs = [docs '    %%                                 3: longitude    [rad]     The longitude measurement.\n'];
                    case 3
                        docs = [docs '    %%                                 2: latitude     [rad]     The latitude measurement.\n'];
                        docs = [docs '    %%                                 3: longitude    [rad]     The longitude measurement.\n'];
                        docs = [docs '    %%                                 4: altitude     [m]       The altitude measurement.\n'];
                end
                docs = [docs '    %% ' arg_Pn_StdDev '                  ... ' num2str(dim) '-by-1 vector representing the standard deviation for the position measurements of position sensor ' num2str(n) '. The elements are as follows:\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                                 1: stdDown      [m]       Standard deviation in up/down-direction.\n'];
                    case 2
                        docs = [docs '    %%                                 1: stdNorth     [m]       Standard deviation in north direction.\n'];
                        docs = [docs '    %%                                 2: stdEast      [m]       Standard deviation in east direction.\n'];
                    case 3
                        docs = [docs '    %%                                 1: stdNorth     [m]       Standard deviation in north direction.\n'];
                        docs = [docs '    %%                                 2: stdEast      [m]       Standard deviation in east direction.\n'];
                        docs = [docs '    %%                                 3: stdDown      [m]       Standard deviation in up/down-direction.\n'];
                end
                docs = [docs '    %% ' arg_Pn_PositionBody2Sensor '     ... Position of position sensor ' num2str(n) ' w.r.t. body frame origin in body frame coordinates (meters).\n'];

                % Variable code section A
                varCodeSectionA = [varCodeSectionA '    assert((' num2str(int32(1) + dim) ' == size(' arg_Pn_Data ',1)) && (1 == size(' arg_Pn_Data ',2)) && (' num2str(dim) ' == size(' arg_Pn_StdDev ',1)) && (1 == size(' arg_Pn_StdDev ',2)) && (3 == size(' arg_Pn_PositionBody2Sensor ',1)) && (1 == size(' arg_Pn_PositionBody2Sensor ',2)));\n'];
                varCodeSectionA = [varCodeSectionA '    persistent ' arg_Pn_Buffer '; if(isempty(' arg_Pn_Buffer ') || (' arg_Pn_Data '(1) > 0.0)), ' arg_Pn_Buffer ' = ' arg_Pn_Data '; end\n'];

                % Variable code section B
                varCodeSectionB = [varCodeSectionB '        if(' arg_Pn_Buffer '(1) > 0.0), numUpdates = numUpdates + int32(1); ' arg_Pn_Buffer '(1) = 0.0; ins.UpdatePosition' num2str(dim) 'D(' arg_Pn_Buffer '(2:end), ' arg_Pn_StdDev ', ' arg_Pn_PositionBody2Sensor '); end\n'];
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
                docs = [docs '    %% ' arg_Vn_Data '                    ... ' num2str(int32(1) + dim) '-by-1 vector representing the velocity measurements of velocity sensor ' num2str(n) '. The elements are as follows:\n'];
                docs = [docs '    %%                                 1: trigger                Greater than 0.0 if new measurement data is available and should be used in an update step, 0.0 otherwise.\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                                 2: velocityZ    [m/s]     The velocity measurement in z-direction of sensor frame.\n'];
                    case 2
                        docs = [docs '    %%                                 2: velocityX    [m/s]     The velocity measurement in x-direction of sensor frame.\n'];
                        docs = [docs '    %%                                 3: velocityY    [m/s]     The velocity measurement in y-direction of sensor frame.\n'];
                    case 3
                        docs = [docs '    %%                                 2: velocityX    [m/s]     The velocity measurement in x-direction of sensor frame.\n'];
                        docs = [docs '    %%                                 3: velocityY    [m/s]     The velocity measurement in y-direction of sensor frame.\n'];
                        docs = [docs '    %%                                 4: velocityZ    [m/s]     The velocity measurement in z-direction of sensor frame.\n'];
                end
                docs = [docs '    %% ' arg_Vn_StdDev '                  ... ' num2str(dim) '-by-1 vector representing the standard deviation for the velocity measurements of velocity sensor ' num2str(n) '. The elements are as follows:\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                                 1: stdZ         [m/s]     Standard deviation in z-direction.\n'];
                    case 2
                        docs = [docs '    %%                                 1: stdX         [m/s]     Standard deviation in x-direction.\n'];
                        docs = [docs '    %%                                 2: stdY         [m/s]     Standard deviation in y-direction.\n'];
                    case 3
                        docs = [docs '    %%                                 1: stdX         [m/s]     Standard deviation in x-direction.\n'];
                        docs = [docs '    %%                                 2: stdY         [m/s]     Standard deviation in y-direction.\n'];
                        docs = [docs '    %%                                 3: stdZ         [m/s]     Standard deviation in z-direction.\n'];
                end
                docs = [docs '    %% ' arg_Vn_PositionBody2Sensor '     ... Position of velocity sensor ' num2str(n) ' w.r.t. body frame origin in body frame coordinates (meters).\n'];
                docs = [docs '    %% ' arg_Vn_DCMBody2Sensor '          ... Direction cosine matrix to rotate vectors from body frame to sensor frame.\n'];

                % Variable code section A
                varCodeSectionA = [varCodeSectionA '    assert((' num2str(int32(1) + dim) ' == size(' arg_Vn_Data ',1)) && (1 == size(' arg_Vn_Data ',2)) && (' num2str(dim) ' == size(' arg_Vn_StdDev ',1)) && (1 == size(' arg_Vn_StdDev ',2)) && (3 == size(' arg_Vn_PositionBody2Sensor ',1)) && (1 == size(' arg_Vn_PositionBody2Sensor ',2)) && (3 == size(' arg_Vn_DCMBody2Sensor ',1)) && (3 == size(' arg_Vn_DCMBody2Sensor ',2)));\n'];
                varCodeSectionA = [varCodeSectionA '    persistent ' arg_Vn_Buffer '; if(isempty(' arg_Vn_Buffer ') || (' arg_Vn_Data '(1) > 0.0)), ' arg_Vn_Buffer ' = ' arg_Vn_Data '; end\n'];

                % Variable code section B
                varCodeSectionB = [varCodeSectionB '        if(' arg_Vn_Buffer '(1) > 0.0), numUpdates = numUpdates + int32(1); ' arg_Vn_Buffer '(1) = 0.0; ins.UpdateVelocity' num2str(dim) 'D(' arg_Vn_Buffer '(2:end), ' arg_Vn_StdDev ', ' arg_Vn_PositionBody2Sensor ', ' arg_Vn_DCMBody2Sensor '); end\n'];
            end

            % Orientation measurements: additional arguments, documentation and variable code sections
            for n = int32(1):numOrientationMeasurements
                dim = orientationMeasurements(n);
                arg_On_Data = ['O' num2str(n) '_Data'];
                arg_On_StdDev = ['O' num2str(n) '_StdDev'];
                arg_On_DCMBody2Sensor = ['O' num2str(n) '_DCMBody2Sensor'];
                arg_On_Buffer = ['O' num2str(n) '_Buffer'];
                args = [args ', ' arg_On_Data ', ' arg_On_StdDev ', ' arg_On_DCMBody2Sensor];
                docs = [docs '    %% ' arg_On_Data '                    ... ' num2str(int32(1) + dim) '-by-1 vector representing the orientation measurements of orientation sensor ' num2str(n) '. Note that orientation measurements must only be used for applications with small roll and pitch angles. The elements are as follows:\n'];
                docs = [docs '    %%                                 1: trigger                Greater than 0.0 if new measurement data is available and should be used in an update step, 0.0 otherwise.\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                                 2: yaw          [rad]     The yaw angle measurement (ZYX-convention).\n'];
                    case 2
                        docs = [docs '    %%                                 2: roll         [rad]     The roll angle measurement (ZYX-convention).\n'];
                        docs = [docs '    %%                                 3: pitch        [rad]     The pitch angle measurement (ZYX-convention).\n'];
                    case 3
                        docs = [docs '    %%                                 2: roll         [rad]     The roll angle measurement (ZYX-convention).\n'];
                        docs = [docs '    %%                                 3: pitch        [rad]     The pitch angle measurement (ZYX-convention).\n'];
                        docs = [docs '    %%                                 4: yaw          [rad]     The yaw angle measurement (ZYX-convention).\n'];
                end
                docs = [docs '    %% ' arg_On_StdDev '                  ... ' num2str(dim) '-by-1 vector representing the standard deviation for the orientation measurements of orientation sensor ' num2str(n) '. The elements are as follows:\n'];
                switch(dim)
                    case 1
                        docs = [docs '    %%                                 1: stdYaw       [rad]     Standard deviation for yaw angle.\n'];
                    case 2
                        docs = [docs '    %%                                 1: stdRoll      [rad]     Standard deviation for roll angle.\n'];
                        docs = [docs '    %%                                 2: stdPitch     [rad]     Standard deviation for pitch angle.\n'];
                    case 3
                        docs = [docs '    %%                                 1: stdRoll      [rad]     Standard deviation for roll angle.\n'];
                        docs = [docs '    %%                                 2: stdPitch     [rad]     Standard deviation for pitch angle.\n'];
                        docs = [docs '    %%                                 3: stdYaw       [rad]     Standard deviation for yaw angle.\n'];
                end
                docs = [docs '    %% ' arg_On_DCMBody2Sensor '          ... Direction cosine matrix to rotate vectors from body frame to sensor frame.\n'];

                % Variable code section A
                varCodeSectionA = [varCodeSectionA '    assert((' num2str(int32(1) + dim) ' == size(' arg_On_Data ',1)) && (1 == size(' arg_On_Data ',2)) && (' num2str(dim) ' == size(' arg_On_StdDev ',1)) && (1 == size(' arg_On_StdDev ',2)) && (3 == size(' arg_On_DCMBody2Sensor ',1)) && (3 == size(' arg_On_DCMBody2Sensor ',2)));\n'];
                varCodeSectionA = [varCodeSectionA '    persistent ' arg_On_Buffer '; if(isempty(' arg_On_Buffer ') || (' arg_On_Data '(1) > 0.0)), ' arg_On_Buffer ' = ' arg_On_Data '; end\n'];

                % Variable code section B
                varCodeSectionB = [varCodeSectionB '        if(' arg_On_Buffer '(1) > 0.0), numUpdates = numUpdates + int32(1); ' arg_On_Buffer '(1) = 0.0; ins.UpdateOrientation' num2str(dim) 'D(' arg_On_Buffer '(2:end), ' arg_On_StdDev ', ' arg_On_DCMBody2Sensor '); end\n'];
            end

            % SOG measurements: additional arguments, documentation and variable code sections
            for n = int32(1):numSOGMeasurements
                arg_SOGn_Data = ['SOG' num2str(n) '_Data'];
                arg_SOGn_StdDev = ['SOG' num2str(n) '_StdDev'];
                arg_SOGn_PositionBody2Sensor = ['SOG' num2str(n) '_PositionBody2Sensor'];
                arg_SOGn_Buffer = ['SOG' num2str(n) '_Buffer'];
                args = [args ', ' arg_SOGn_Data ', ' arg_SOGn_StdDev ', ' arg_SOGn_PositionBody2Sensor];
                docs = [docs '    %% ' arg_SOGn_Data '                  ... 2-by-1 vector representing the speed-over-ground measurements of SOG sensor ' num2str(n) '. The elements are as follows:\n'];
                docs = [docs '    %%                                 1: trigger                Greater than 0.0 if new measurement data is available and should be used in an update step, 0.0 otherwise.\n'];
                docs = [docs '    %%                                 2: speedOverGround [m/s]     The velocity measurement in z-direction of sensor frame.\n'];
                docs = [docs '    %% ' arg_SOGn_StdDev '                ... Scalar value representing the standard deviation for the speed-over-ground measurement of SOG sensor ' num2str(n) ' in [m/s].\n'];
                docs = [docs '    %% ' arg_SOGn_PositionBody2Sensor '   ... Position of SOG sensor ' num2str(n) ' w.r.t. body frame origin in body frame coordinates (meters).\n'];

                % Variable code section A
                varCodeSectionA = [varCodeSectionA '    assert((2 == size(' arg_SOGn_Data ',1)) && (1 == size(' arg_SOGn_Data ',2)) && (1 == size(' arg_SOGn_StdDev ',1)) && (1 == size(' arg_SOGn_StdDev ',2)) && (3 == size(' arg_SOGn_PositionBody2Sensor ',1)) && (1 == size(' arg_SOGn_PositionBody2Sensor ',2)));\n'];
                varCodeSectionA = [varCodeSectionA '    persistent ' arg_SOGn_Buffer '; if(isempty(' arg_SOGn_Buffer ') || (' arg_SOGn_Data '(1) > 0.0)), ' arg_SOGn_Buffer ' = ' arg_SOGn_Data '; end\n'];

                % Variable code section B
                varCodeSectionB = [varCodeSectionB '        if(' arg_SOGn_Buffer '(1) > 0.0), numUpdates = numUpdates + int32(1); ' arg_SOGn_Buffer '(1) = 0.0; ins.UpdateSOG(' arg_SOGn_Buffer '(2:end), ' arg_SOGn_StdDev ', ' arg_SOGn_PositionBody2Sensor '); end\n'];
            end

            % Return values
            docs = [docs '    %% \n    %% RETURN'];
            docs = [docs '\n    %% valid                      ... A scalar boolean that indicates whether the motion state is valid or not. It is valid if the INS was initialized.'];
            docs = [docs '\n    %% numPredictions             ... An integer indicating the number of prediction steps that have been calculated (either 0 or 1).'];
            docs = [docs '\n    %% numUpdates                 ... An integer indicating the number of update steps that have been calculated.'];
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
            docs = [docs '\n    %% internalX                  ... 10-by-1 state vector representing the internal motion state with respect to the IMU. Elements are as follows:'];
            docs = [docs '\n    %%                            ...  1: Latitude in radians.'];
            docs = [docs '\n    %%                            ...  2: Longitude in radians.'];
            docs = [docs '\n    %%                            ...  3: Altitude in meters (positive upwards).'];
            docs = [docs '\n    %%                            ...  4: North velocity in meters per second.'];
            docs = [docs '\n    %%                            ...  5: East velocity in meters per second.'];
            docs = [docs '\n    %%                            ...  6: Down velocity in meters per second.'];
            docs = [docs '\n    %%                            ...  7: Scalar part of unit quaternion.'];
            docs = [docs '\n    %%                            ...  8: First element of vector part of unit quaternion.'];
            docs = [docs '\n    %%                            ...  9: Second element of vector part of unit quaternion.'];
            docs = [docs '\n    %%                            ... 10: Third element of vector part of unit quaternion.'];
            docs = [docs '\n    %% internalS                  ... 9-by-9 matrix representing the internal square root of the state covariance matrix. This matrix relates to the internalX state. Note that the uncertainty of the quaternion (4D) is represented by an orientation vector (3D).'];

            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            % Generate the final function code
            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            code = ['function [valid, numPredictions, numUpdates, positionLLA, orientationQuaternionWXYZ, orientationRollPitchYaw, velocityNED, velocityUVW, velocityPQR, courseOverGround, speedOverGround, angleOfAttack, sideSlipAngle, internalX, internalS] = ' functionName '(time, ' args ')\n' docs '\n'];
            code = [code '    assert(isscalar(time) && (9 == size(initialState,1)) && (1 == size(initialState,2)) && (9 == size(initialStdDev,1)) && (1 == size(initialStdDev,2)) && isscalar(reset) && isscalar(sampletime) && (7 == size(IMU_Data,1)) && (1 == size(IMU_Data,2)) && (3 == size(IMU_PositionBody2Sensor,1)) && (1 == size(IMU_PositionBody2Sensor,2)) && (3 == size(IMU_DCMBody2Sensor,1)) && (3 == size(IMU_DCMBody2Sensor,2)));\n'];
            code = [code '    persistent prevTime; if(isempty(prevTime)), prevTime = time; end\n'];
            code = [code '    persistent ins; if(isempty(ins)), ins = GenericINS.SensorFusionNoBiasEstimation(); end\n'];
            code = [code '    persistent imuStdDevBuffer; if(isempty(imuStdDevBuffer)), imuStdDevBuffer = IMU_StdDev; end\n\n'];
            code = [code varCodeSectionA '\n'];
            code = [code '    imuStdDevChanged = (sum(abs(imuStdDevBuffer - IMU_StdDev) > 100*eps) > 0.0);\n'];
            code = [code '    eventReset = (reset > 0.0);\n'];
            code = [code '    if(eventReset)\n'];
            code = [code '        ins.Initialize(initialState(1:3), initialState(4:6), initialState(7:9), initialStdDev(1:3), initialStdDev(4:6), initialStdDev(7:9), IMU_StdDev(1:3), IMU_StdDev(4:6));\n'];
            code = [code '    elseif(imuStdDevChanged)\n'];
            code = [code '        ins.ResetInertialStandardDeviation(IMU_StdDev(1:3), IMU_StdDev(4:6));\n'];
            code = [code '    end\n'];
            code = [code '    imuStdDevBuffer = IMU_StdDev;\n'];
            code = [code '    numPredictions = int32(0);\n'];
            code = [code '    numUpdates = int32(0);\n'];
            code = [code '    if((IMU_Data(1) > 0.0) && ~eventReset)\n'];
            code = [code '        prevTime = time;\n\n'];
            code = [code '        %% Motion prediction\n'];
            code = [code '        numPredictions = int32(1);\n'];
            code = [code '        ins.Predict(IMU_Data(2:7), sampletime, IMU_DCMBody2Sensor, IMU_PositionBody2Sensor);\n\n'];
            code = [code '        %% Sensor updates\n'];
            code = [code varCodeSectionB];
            code = [code '    end\n\n'];
            code = [code '    %% Check for timeout\n'];
            code = [code '    if((time - prevTime) > 0.2)\n'];
            code = [code '        ins.MakeInvalid();\n'];
            code = [code '    end\n\n'];
            code = [code '    %% Set output values\n'];
            code = [code '    [valid, positionLLA, orientationQuaternionWXYZ, orientationRollPitchYaw, velocityNED, velocityUVW, velocityPQR, courseOverGround, speedOverGround, angleOfAttack, sideSlipAngle] = ins.GetMotionState();\n'];
            code = [code '    [internalX, internalS] = ins.GetInternalState();\n'];
            code = [code '    internalX = reshape(internalX, [10 1]);\n'];
            code = [code '    internalS = reshape(internalS, [9 9]);\n'];
            code = [code 'end\n\n'];

            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            % Write code to a text file
            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            filename = [functionName '.m'];
            fileID = fopen(filename,'w');
            assert(-1 ~= fileID, ['GenericINS.SensorFusionNoBiasEstimation.GenerateFunction(): Could not open file "' filename '"!']);
            fprintf(fileID, code);
            fclose(fileID);
        end
    end
    methods
        function obj = SensorFusionNoBiasEstimation()
            % Initialize private properties
            obj.initialized = false;
            obj.firstPrediction = false;
            obj.x = zeros(GenericINS.SensorFusionNoBiasEstimation.DIM_L, 1);
            obj.x(7) = 1.0;
            obj.S = zeros(GenericINS.SensorFusionNoBiasEstimation.DIM_LS);
            obj.Xi = zeros(GenericINS.SensorFusionNoBiasEstimation.DIM_L, GenericINS.SensorFusionNoBiasEstimation.NUM_SP);
            obj.Xi(7,:) = ones(1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP);
            obj.dX = zeros(GenericINS.SensorFusionNoBiasEstimation.DIM_XS, GenericINS.SensorFusionNoBiasEstimation.NUM_SP);
            obj.spU2D = false;
            obj.gyrRaw = zeros(3, 1);
            obj.dcmIMUBody2Sensor = eye(3);
            obj.posIMUBody2Sensor = zeros(3, 1);
            obj.w0 = 0.1;
            obj.wi = (1.0 - obj.w0) / (double(GenericINS.SensorFusionNoBiasEstimation.DIM_LS) + 1.0);
            obj.srw0 = sqrt(obj.w0);
            obj.srwi = sqrt(obj.wi);

            % Initialize vector sequence for weighting matrix Z
            obj.Z = zeros(GenericINS.SensorFusionNoBiasEstimation.DIM_LS, GenericINS.SensorFusionNoBiasEstimation.NUM_SP);
            obj.Z(1,3) = 1.0 / sqrt(2.0 * obj.wi);
            obj.Z(1,2) = -obj.Z(1,3);

            % Expand vector sequence
            for j = int32(2):GenericINS.SensorFusionNoBiasEstimation.DIM_LS
                s = 1.0 / sqrt(double(j * (j + 1)) * obj.wi);
                obj.Z(j, j + 2) = double(j) * s;
                for k = int32(2):(j + 1)
                    obj.Z(j, k) = -s;
                end
            end
        end
        function Initialize(obj, initialPositionLLA, initialVelocityUVW, initialOrientationRollPitchYaw, initialStdPositionLLA, initialStdVelocityUVW, initialStdOrientationVector, stdAcc, stdGyr)
            %GenericINS.SensorFusionNoBiasEstimation.Initialize Initialize or re-initialize the INS.
            % 
            % PARAMETERS
            % initialPositionLLA             ... Initial geographic position at the location of the IMU, [lat (rad); lon (rad); alt (m, positive upwards)].
            % initialVelocityUVW             ... Initial body-fixed velocity at the location of the IMU, [u (m/s); v (m/s); r (m/s)].
            % initialOrientationRollPitchYaw ... Initial orientation of the body frame, [roll (rad); pitch (rad); yaw (rad)].
            % initialStdPositionLLA          ... Initial standard deviation for the geographic position. Should be an estimate of the standard deviation of initialPositionLLA.
            % initialStdVelocityUVW          ... Initial standard deviation for the body-fixed velocity. Should be an estimate of the standard deviation of initialVelocityUVW.
            % initialStdOrientationVector    ... Initial standard deviation for the orientation vector (axis-angle).
            % stdAcc                         ... Standard deviation for acceleration in m/s^2 to be used for motion prediction.
            % stdGyr                         ... Standard deviation for angular rate in rad/s to be used for motion prediction.
            assert((3 == size(initialPositionLLA,1)) && (1 == size(initialPositionLLA,2)) && isa(initialPositionLLA, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Initialize(): "initialPositionLLA" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialVelocityUVW,1)) && (1 == size(initialVelocityUVW,2)) && isa(initialVelocityUVW, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Initialize(): "initialVelocityUVW" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialOrientationRollPitchYaw,1)) && (1 == size(initialOrientationRollPitchYaw,2)) && isa(initialOrientationRollPitchYaw, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Initialize(): "initialOrientationRollPitchYaw" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialStdPositionLLA,1)) && (1 == size(initialStdPositionLLA,2)) && isa(initialStdPositionLLA, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Initialize(): "initialStdPositionLLA" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialStdVelocityUVW,1)) && (1 == size(initialStdVelocityUVW,2)) && isa(initialStdVelocityUVW, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Initialize(): "initialStdVelocityUVW" must be a 3-by-1 vector of type double!');
            assert((3 == size(initialStdOrientationVector,1)) && (1 == size(initialStdOrientationVector,2)) && isa(initialStdOrientationVector, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Initialize(): "initialStdOrientationVector" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdAcc,1)) && (1 == size(stdAcc,2)) && isa(stdAcc, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Initialize(): "stdAcc" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdGyr,1)) && (1 == size(stdGyr,2)) && isa(stdGyr, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Initialize(): "stdGyr" must be a 3-by-1 vector of type double!');

            % Initial position
            [lat, lon] = GenericINS.SensorFusionNoBiasEstimation.LatLon(initialPositionLLA(1), initialPositionLLA(2));
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
            DCM_b2n = GenericINS.SensorFusionNoBiasEstimation.Cb2n(obj.x(7:10));
            obj.x(4:6) = DCM_b2n * initialVelocityUVW;

            % Initial square root covariance matrix
            obj.S = diag([initialStdPositionLLA; DCM_b2n * initialStdVelocityUVW; initialStdOrientationVector; stdAcc; stdGyr]);

            % Initialize additional INS properties
            obj.spU2D = false;
            obj.gyrRaw = zeros(3,1);
            obj.initialized = true;
            obj.firstPrediction = false;
        end
        function ResetInertialStandardDeviation(obj, stdAcc, stdGyr)
            %GenericINS.SensorFusionNoBiasEstimation.ResetInertialStandardDeviation Reset the standard deviations for the inertial sensors.
            % 
            % PARAMETERS
            % stdAcc            ... Standard deviation for acceleration in m/s^2.
            % stdGyr            ... Standard deviation for angular rate in rad/s.
            assert((3 == size(stdAcc,1)) && (1 == size(stdAcc,2)) && isa(stdAcc, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Predict(): "stdAcc" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdGyr,1)) && (1 == size(stdGyr,2)) && isa(stdGyr, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Predict(): "stdGyr" must be a 3-by-1 vector of type double!');
            obj.S((GenericINS.SensorFusionNoBiasEstimation.DIM_XS + int32(1)):end,(GenericINS.SensorFusionNoBiasEstimation.DIM_XS + int32(1)):end) = diag([stdAcc; stdGyr]);
        end
        function Predict(obj, imuData, sampletime, dcmIMUBody2Sensor, posIMUBody2Sensor)
            %GenericINS.SensorFusionNoBiasEstimation.Predict Perform a prediction step using inertial measurements. The prediction is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % imuData           ... Inertial measurement data vector: [accX (m/s^2); accY (m/s^2); accZ (m/s^2); gyrX (rad/s); gyrY (rad/s); gyrZ (rad/s)]
            % sampletime        ... Discrete sampletime in seconds to be used for forward euler integration. Should be the elapsed time to the latest prediction.
            % dcmIMUBody2Sensor ... Direction cosine matrix to rotate measurements from body frame to sensor frame.
            % posIMUBody2Sensor ... Position of IMU sensor frame origin w.r.t. body frame origin in body frame coordinates (meters).
            assert((6 == size(imuData,1)) && (1 == size(imuData,2)) && isa(imuData, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Predict(): "imuData" must be a 6-by-1 vector of type double!');
            assert(isscalar(sampletime) && isa(sampletime, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Predict(): "sampletime" must be a scalar of type double!');
            assert((3 == size(dcmIMUBody2Sensor,1)) && (3 == size(dcmIMUBody2Sensor,2)) && isa(dcmIMUBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Predict(): "dcmIMUBody2Sensor" must be a 3-by-3 matrix of type double!');
            assert((3 == size(posIMUBody2Sensor,1)) && (1 == size(posIMUBody2Sensor,2)) && isa(posIMUBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.Predict(): "posIMUBody2Sensor" must be a 3-by-1 vector of type double!');

            % Do not allow prediction of an uninitialized INS
            if(~obj.initialized)
                return;
            end
            obj.firstPrediction = true;

            % Generate sigma points
            obj.GenerateSigmaPoints();
            obj.spU2D = true;

            % Propagate through process model
            for i = int32(1):GenericINS.SensorFusionNoBiasEstimation.NUM_SP
                state = obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i);
                w = obj.Xi((GenericINS.SensorFusionNoBiasEstimation.DIM_X+int32(1)):(GenericINS.SensorFusionNoBiasEstimation.DIM_X + GenericINS.SensorFusionNoBiasEstimation.DIM_W),i);
                obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i) = GenericINS.SensorFusionNoBiasEstimation.ProcessModelEuler(state, w, imuData, sampletime, dcmIMUBody2Sensor);
            end

            % Calculate dX = Xi - x
            obj.CalculateDX();

            % Update square root covariance
            [~, Sx_UPPER] = qr((obj.srwi*obj.dX(:,2:GenericINS.SensorFusionNoBiasEstimation.NUM_SP))', 0);
            Sx_UPPER = cholupdate(Sx_UPPER, obj.srw0*obj.dX(:,1), '+');
            obj.S(int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS,int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS) = Sx_UPPER';

            % Save latest IMU data
            obj.gyrRaw = imuData(4:6);
            obj.dcmIMUBody2Sensor = dcmIMUBody2Sensor;
            obj.posIMUBody2Sensor = posIMUBody2Sensor;
        end
        function UpdatePosition3D(obj, measurementLatLonAlt, stdNorthEastDown, posBody2Sensor)
            %GenericINS.SensorFusionNoBiasEstimation.UpdatePosition3D Perform an observation step for 3D geographic position data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementLatLonAlt ... The measurement vector [lat (rad); lon (rad); alt (m, positive upwards)].
            % stdNorthEastDown     ... The standard deviation in meters, [north; east; down].
            % posBody2Sensor       ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            assert((3 == size(measurementLatLonAlt,1)) && (1 == size(measurementLatLonAlt,2)) && isa(measurementLatLonAlt, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdatePosition3D(): "measurementLatLonAlt" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdNorthEastDown,1)) && (1 == size(stdNorthEastDown,2)) && isa(stdNorthEastDown, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdatePosition3D(): "stdNorthEastDown" must be a 3-by-1 vector of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdatePosition3D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            obj.UpdatePosition(3, measurementLatLonAlt, stdNorthEastDown, posBody2Sensor);
        end
        function UpdatePosition2D(obj, measurementLatLon, stdNorthEast, posBody2Sensor)
            %GenericINS.SensorFusionNoBiasEstimation.UpdatePosition2D Perform an observation step for 2D geographic position data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementLatLon ... The measurement vector [lat (rad); lon (rad)].
            % stdNorthEast      ... The standard deviation in meters, [north; east].
            % posBody2Sensor    ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            assert((2 == size(measurementLatLon,1)) && (1 == size(measurementLatLon,2)) && isa(measurementLatLon, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdatePosition2D(): "measurementLatLon" must be a 2-by-1 vector of type double!');
            assert((2 == size(stdNorthEast,1)) && (1 == size(stdNorthEast,2)) && isa(stdNorthEast, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdatePosition2D(): "stdNorthEast" must be a 2-by-1 vector of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdatePosition2D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            obj.UpdatePosition(2, measurementLatLon, stdNorthEast, posBody2Sensor);
        end
        function UpdatePosition1D(obj, measurementAlt, stdDown, posBody2Sensor)
            %GenericINS.SensorFusionNoBiasEstimation.UpdatePosition1D Perform an observation step for 1D geographic position data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementAlt ... The measurement scalar [alt (m, positive upwards)].
            % stdDown        ... The standard deviation in meters, [down].
            % posBody2Sensor ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            assert(isscalar(measurementAlt) && isa(measurementAlt, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdatePosition1D(): "measurementAlt" must be a scalar of type double!');
            assert(isscalar(stdDown) && isa(stdDown, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdatePosition1D(): "stdDown" must be a scalar of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdatePosition1D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            obj.UpdatePosition(1, measurementAlt, stdDown, posBody2Sensor);
        end
        function UpdateVelocity3D(obj, measurementXYZ, stdXYZ, posBody2Sensor, dcmBody2Sensor)
            %GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity3D Perform an observation step for 3D velocity data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementXYZ ... The measurement vector, [x (m/s); y (m/s); z (m/s)].
            % stdXYZ         ... The standard deviation, [x (m/s); y (m/s); z (m/s)].
            % posBody2Sensor ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            % dcmBody2Sensor ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert((3 == size(measurementXYZ,1)) && (1 == size(measurementXYZ,2)) && isa(measurementXYZ, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity3D(): "measurementXYZ" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdXYZ,1)) && (1 == size(stdXYZ,2)) && isa(stdXYZ, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity3D(): "stdXYZ" must be a 3-by-1 vector of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity3D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity3D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateVelocity(3, measurementXYZ, stdXYZ, posBody2Sensor, dcmBody2Sensor);
        end
        function UpdateVelocity2D(obj, measurementXY, stdXY, posBody2Sensor, dcmBody2Sensor)
            %GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity2D Perform an observation step for 2D velocity data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementXY  ... The measurement vector, [x (m/s); y (m/s)].
            % stdXY          ... The standard deviation, [x (m/s); y (m/s)].
            % posBody2Sensor ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            % dcmBody2Sensor ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert((2 == size(measurementXY,1)) && (1 == size(measurementXY,2)) && isa(measurementXY, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity2D(): "measurementXY" must be a 2-by-1 vector of type double!');
            assert((2 == size(stdXY,1)) && (1 == size(stdXY,2)) && isa(stdXY, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity2D(): "stdXY" must be a 2-by-1 vector of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity2D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity2D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateVelocity(2, measurementXY, stdXY, posBody2Sensor, dcmBody2Sensor);
        end
        function UpdateVelocity1D(obj, measurementZ, stdZ, posBody2Sensor, dcmBody2Sensor)
            %GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity1D Perform an observation step for 1D velocity data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementZ   ... The measurement scalar, [z (m/s)].
            % stdZ           ... The standard deviation, [z (m/s)].
            % posBody2Sensor ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            % dcmBody2Sensor ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert(isscalar(measurementZ) && isa(measurementZ, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity1D(): "measurementZ" must be a scalar of type double!');
            assert(isscalar(stdZ) && isa(stdZ, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity1D(): "stdZ" must be a scalar of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity1D(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateVelocity1D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateVelocity(1, measurementZ, stdZ, posBody2Sensor, dcmBody2Sensor);
        end
        function UpdateSOG(obj, measurementSOG, stdSOG, posBody2Sensor)
            %GenericINS.SensorFusionNoBiasEstimation.UpdateSOG Perform an observation step for speed-over-ground data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementSOG ... The measurement scalar, [sog (m/s)].
            % stdSOG         ... The standard deviation, [sog (m/s)].
            % posBody2Sensor ... Position of sensor frame origin w.r.t. body frame origin in body frame coordinates.
            assert(isscalar(measurementSOG) && isa(measurementSOG, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateSOG(): "measurementSOG" must be a scalar of type double!');
            assert(isscalar(stdSOG) && isa(stdSOG, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateSOG(): "stdSOG" must be a scalar of type double!');
            assert((3 == size(posBody2Sensor,1)) && (1 == size(posBody2Sensor,2)) && isa(posBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateSOG(): "posBody2Sensor" must be a 3-by-1 vector of type double!');
            obj.UpdateSpeedOverGround(measurementSOG, stdSOG, posBody2Sensor);
        end
        function UpdateOrientation3D(obj, measurementRollPitchYaw, stdRollPitchYaw, dcmBody2Sensor)
            %GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation3D Perform an observation step for 3D orientation data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementRollPitchYaw ... The measurement vector, [roll (rad); pitch (rad); yaw (rad)].
            % stdRollPitchYaw         ... The standard deviation, [roll (rad); pitch (rad); yaw (rad)].
            % dcmBody2Sensor          ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert((3 == size(measurementRollPitchYaw,1)) && (1 == size(measurementRollPitchYaw,2)) && isa(measurementRollPitchYaw, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation3D(): "measurementRollPitchYaw" must be a 3-by-1 vector of type double!');
            assert((3 == size(stdRollPitchYaw,1)) && (1 == size(stdRollPitchYaw,2)) && isa(stdRollPitchYaw, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation3D(): "stdRollPitchYaw" must be a 3-by-1 vector of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation3D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateOrientation(3, measurementRollPitchYaw, stdRollPitchYaw, dcmBody2Sensor);
        end
        function UpdateOrientation2D(obj, measurementRollPitch, stdRollPitch, dcmBody2Sensor)
            %GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation2D Perform an observation step for 2D orientation data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementRollPitch ... The measurement vector, [roll (rad); pitch (rad)].
            % stdRollPitch         ... The standard deviation, [roll (rad); pitch (rad)].
            % dcmBody2Sensor       ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert((2 == size(measurementRollPitch,1)) && (1 == size(measurementRollPitch,2)) && isa(measurementRollPitch, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation2D(): "measurementRollPitch" must be a 2-by-1 vector of type double!');
            assert((2 == size(stdRollPitch,1)) && (1 == size(stdRollPitch,2)) && isa(stdRollPitch, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation2D(): "stdRollPitch" must be a 2-by-1 vector of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation2D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateOrientation(2, measurementRollPitch, stdRollPitch, dcmBody2Sensor);
        end
        function UpdateOrientation1D(obj, measurementYaw, stdYaw, dcmBody2Sensor)
            %GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation1D Perform an observation step for 1D orientation data. The update is only calculated if the INS is initialized.
            % 
            % PARAMETERS
            % measurementYaw ... The measurement scalar, [yaw (rad)].
            % stdYaw         ... The standard deviation, [yaw (rad)].
            % dcmBody2Sensor ... Direction cosine matrix to rotate vectors from body frame to sensor frame.
            assert(isscalar(measurementYaw) && isa(measurementYaw, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation1D(): "measurementYaw" must be a scalar of type double!');
            assert(isscalar(stdYaw) && isa(stdYaw, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation1D(): "stdYaw" must be a scalar of type double!');
            assert((3 == size(dcmBody2Sensor,1)) && (3 == size(dcmBody2Sensor,2)) && isa(dcmBody2Sensor, 'double'), 'GenericINS.SensorFusionNoBiasEstimation.UpdateOrientation1D(): "dcmBody2Sensor" must be a 3-by-3 matrix of type double!');
            obj.UpdateOrientation(1, measurementYaw, stdYaw, dcmBody2Sensor);
        end
        function [valid, positionLLA, orientationQuaternionWXYZ, orientationRollPitchYaw, velocityNED, velocityUVW, velocityPQR, courseOverGround, speedOverGround, angleOfAttack, sideSlipAngle] = GetMotionState(obj)
            %GenericINS.SensorFusionNoBiasEstimation.GetMotionState Get the motion state.
            % 
            % RETURN
            % valid                      ... A scalar boolean that indicates if the motion state is valid or not. It is valid if the INS was initialized and performed at least one prediction.
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
            valid = obj.initialized && obj.firstPrediction;

            % WGS84 properties
            [Rn, Re, ~] = GenericINS.SensorFusionNoBiasEstimation.WGS84(obj.x(1));

            % Orientation
            orientationQuaternionWXYZ = GenericINS.SensorFusionNoBiasEstimation.Normalize(obj.x(7:10));
            Cq = GenericINS.SensorFusionNoBiasEstimation.Cb2n(orientationQuaternionWXYZ);
            [roll, pitch, yaw] = GenericINS.SensorFusionNoBiasEstimation.EulerZYX(Cq);
            orientationRollPitchYaw = [roll; pitch; yaw];

            % Geographic position (add vector from accelerometer to body origin)
            iM = diag([(1/(Rn + obj.x(3))); (1/((Re + obj.x(3))*cos(obj.x(1)))); -1.0]);
            positionLLA = obj.x(1:3) - iM * Cq * obj.posIMUBody2Sensor;
            [lat, lon] = GenericINS.SensorFusionNoBiasEstimation.LatLon(positionLLA(1), positionLLA(2));
            positionLLA(1) = lat;
            positionLLA(2) = lon;

            % Rotational velocity (remove bias and align to body frame)
            w_ie = [GenericINS.SensorFusionNoBiasEstimation.OMEGA_EARTH * cos(obj.x(1)); 0; -GenericINS.SensorFusionNoBiasEstimation.OMEGA_EARTH * sin(obj.x(1))];
            w_en = [(obj.x(5) / ((Re + obj.x(3)))); -(obj.x(4) / (Rn + obj.x(3))); -(obj.x(5) / (Re + obj.x(3)))*tan(obj.x(1))];
            velocityPQR = obj.dcmIMUBody2Sensor' * obj.gyrRaw - Cq' * (w_ie + w_en);

            % Velocity in the navigation frame
            velocityNED = obj.x(4:6) - Cq * cross(velocityPQR, obj.posIMUBody2Sensor);

            % Velocity in the body frame
            velocityUVW = Cq' * velocityNED;

            % COG, SOG, AOA, SSA
            courseOverGround = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(atan2(velocityNED(2), velocityNED(1)));
            speedOverGround = sqrt(velocityNED(1:2)'*velocityNED(1:2));
            angleOfAttack = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(atan2(velocityNED(3), speedOverGround));
            sideSlipAngle = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(courseOverGround - yaw);
        end
        function [x, S] = GetInternalState(obj)
            %GenericINS.SensorFusionNoBiasEstimation.GetInternalState Get the internal state of the filter.
            % 
            % RETURN
            % x ... 16-by-1 state vector representing the motion state with respect to the IMU. Elements are as follows
            %        1: Latitude in radians.
            %        2: Longitude in radians.
            %        3: Altitude in meters (positive upwards).
            %        4: North velocity in meters per second.
            %        5: East velocity in meters per second.
            %        6: Down velocity in meters per second.
            %        7: Scalar part of unit quaternion.
            %        8: First element of vector part of unit quaternion.
            %        9: Second element of vector part of unit quaternion.
            %       10: Third element of vector part of unit quaternion.
            % S ... 9-by-9 matrix representing the square root of the state covariance matrix. Note that the uncertainty of the quaternion (4D) is represented by an orientation vector (3D).
            x = obj.x(int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_X);
            S = obj.S(int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS,int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS);
        end
        function MakeInvalid(obj)
            %GenericINS.SensorFusionNoBiasEstimation.MakeInvalid Make the filter status invalid. An initialization followed by at least one prediction step
            % is required to make the filter valid again.
            obj.initialized = false;
            obj.firstPrediction = false;
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
            obj.Xi = repmat(obj.x, 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP);

            % For remaining sigma points S*Z is required
            SZ = obj.S * obj.Z(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP);

            % Convert orientation-vector components (3 dimensions) to quaternion-components (4 dimensions): SZq will have one more row than SZ
            SZq = [SZ(1:6,:); zeros(4,GenericINS.SensorFusionNoBiasEstimation.NUM_SP - int32(1)); SZ(10:end,:)];
            SZq(7:10,:) = GenericINS.SensorFusionNoBiasEstimation.OV2Q(SZ(7:9,:));

            % Calculate final sigma points by Xi = x + S*Z, however, with special treatment for quaternion parts (quaternion multiplication instead of vector addition)
            % We leave Xi(:,1) as it is because the 0-th sigma point is equal to the mean value
            obj.Xi(1:6,2:end)    = obj.Xi(1:6,2:end) + SZq(1:6,:);
            obj.Xi(7:10,2:end)   = GenericINS.SensorFusionNoBiasEstimation.Qdot(SZq(7:10,:), obj.Xi(7:10,1)); 
            obj.Xi(11:end,2:end) = obj.Xi(11:end,2:end) + SZq(11:end,:);
        end
        function CalculateDX(obj)
            % Get the pivot angle from 0-th sigma point and remove this angle from all sigma points
            pivotAngle = obj.Xi(2,1);
            obj.Xi(2,:) = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(obj.Xi(2,:) - repmat(pivotAngle, 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP));

            % Calculate weighted mean of sigma points (barycentric mean for quaternion part)
            obj.x(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X) = obj.w0 * obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,1) + obj.wi * sum(obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,2:GenericINS.SensorFusionNoBiasEstimation.NUM_SP), 2);
            obj.x(7:10) = GenericINS.SensorFusionNoBiasEstimation.Normalize(obj.x(7:10));

            % Calculate dX = Xi - x
            % Special treatment for angle and quaternion. Xi - x: We need the inverse quaternion because of -x.
            q_inv = [obj.x(7); -obj.x(8:10)];
            dXq = GenericINS.SensorFusionNoBiasEstimation.Qdot(obj.Xi(7:10,:), q_inv);
            obj.dX = [obj.Xi(1,:) - repmat(obj.x(1), 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP); ...
                  GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(obj.Xi(2,:) - repmat(obj.x(2), 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP)); ...
                  obj.Xi(3:6,:) - repmat(obj.x(3:6), 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP); ...
                  GenericINS.SensorFusionNoBiasEstimation.Q2OV(dXq)];

            % Add pivot angle to state (and also sigma-points for an upcomming measurement update)
            obj.x(2) = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(obj.x(2) + pivotAngle);
            obj.Xi(2,:) = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(obj.Xi(2,:) + repmat(pivotAngle, 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP));
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
            Y = zeros(yDim, GenericINS.SensorFusionNoBiasEstimation.NUM_SP);
            switch(yDim)
                case 1
                    for i = int32(1):GenericINS.SensorFusionNoBiasEstimation.NUM_SP
                        state = obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i);
                        y = GenericINS.SensorFusionNoBiasEstimation.SensorModelPosition(state, obj.posIMUBody2Sensor, posBody2Sensor);
                        Y(:,i) = y(3);
                    end
                case 2
                    for i = int32(1):GenericINS.SensorFusionNoBiasEstimation.NUM_SP
                        state = obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i);
                        y = GenericINS.SensorFusionNoBiasEstimation.SensorModelPosition(state, obj.posIMUBody2Sensor, posBody2Sensor);
                        Y(:,i) = y(1:2);
                    end
                case 3
                    for i = int32(1):GenericINS.SensorFusionNoBiasEstimation.NUM_SP
                        state = obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i);
                        Y(:,i) = GenericINS.SensorFusionNoBiasEstimation.SensorModelPosition(state, obj.posIMUBody2Sensor, posBody2Sensor);
                    end
            end

            % Get pivot angle from 0-th Y sigma-point and remove this angle from all sigma-points
            pivotAngle = 0;
            if(yDim > int32(1))
                pivotAngle = Y(2,1);
                Y(2,:) = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(Y(2,:) - repmat(pivotAngle, 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP));
            end

            % Calculate weighted mean of sigma-points
            ym = obj.w0 * Y(:,1) + obj.wi * sum(Y(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP), 2);

            % Calculate dY = Y - ym
            dY = (Y - repmat(ym, 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP));

            % Add pivot angle to mean value
            if(yDim > int32(1))
                ym(2) = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(ym(2) + pivotAngle);
            end

            % Calculate the innovation
            innovation = measurement - ym;
            if(yDim > int32(1))
                innovation(2) = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(innovation(2));
            end

            % Calculate uncertainty: [m]->[rad] for lat,lon
            [Rn, Re, ~] = GenericINS.SensorFusionNoBiasEstimation.WGS84(obj.x(1));
            sqrtR = diag(stdMeasurement);
            if(yDim > int32(1))
                sqrtR(1,1) = sqrtR(1,1) / (Rn + obj.x(3));
                sqrtR(2,2) = sqrtR(2,2) / ((Re + obj.x(3)) * cos(obj.x(1)));
            end

            % QR decomposition of sigma points: additive SRSSUKF
            [~, Sy_UPPER] = qr([(obj.srwi * dY(:,2:GenericINS.SensorFusionNoBiasEstimation.NUM_SP)) sqrtR]', 0);

            % Cholesky update or downdate (w0 is guaranteed to be positive -> update)
            Sy_UPPER = cholupdate(Sy_UPPER, obj.srw0 * dY(:,1), '+');
            Sy = Sy_UPPER';

            % Calculate covariance matrix Pxy
            Pxy = obj.w0 * obj.dX(:,1) * dY(:,1)' + obj.wi * obj.dX(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP) * dY(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP)';

            % Calculate the Kalman gain matrix
            K = (Pxy / Sy') / Sy;

            % Get state update dx = K * innovation
            dx = K * innovation;

            % Update state estimation x = x + dx taking angles and quaternions into account
            obj.x(1:6) = obj.x(1:6) + dx(1:6);
            obj.x(2) = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(obj.x(2));
            obj.x(7:10) = GenericINS.SensorFusionNoBiasEstimation.Qdot(GenericINS.SensorFusionNoBiasEstimation.OV2Q(dx(7:9)), obj.x(7:10));

            % Update sqrt of covariance using cholesky downdate. Because MATLABs cholesky update uses the UPPER triangle we have to transpose Sx.
            Sx_UPPER = obj.S(int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS,int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS)';
            U = K * Sy;
            for j = 1:yDim
                [Sx_UPPER,~] = cholupdate(Sx_UPPER, U(:,j), '-');
            end

            % Convert from UPPER cholesky factor to LOWER cholesky factor
            obj.S(int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS,int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS) = Sx_UPPER';
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
            Y = zeros(yDim, GenericINS.SensorFusionNoBiasEstimation.NUM_SP);
            switch(yDim)
                case 1
                    for i = int32(1):GenericINS.SensorFusionNoBiasEstimation.NUM_SP
                        state = obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i);
                        y = GenericINS.SensorFusionNoBiasEstimation.SensorModelVelocity(state, obj.gyrRaw, obj.posIMUBody2Sensor, obj.dcmIMUBody2Sensor, posBody2Sensor, dcmBody2Sensor);
                        Y(:,i) = y(3);
                    end
                case 2
                    for i = int32(1):GenericINS.SensorFusionNoBiasEstimation.NUM_SP
                        state = obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i);
                        y = GenericINS.SensorFusionNoBiasEstimation.SensorModelVelocity(state, obj.gyrRaw, obj.posIMUBody2Sensor, obj.dcmIMUBody2Sensor, posBody2Sensor, dcmBody2Sensor);
                        Y(:,i) = y(1:2);
                    end
                case 3
                    for i = int32(1):GenericINS.SensorFusionNoBiasEstimation.NUM_SP
                        state = obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i);
                        Y(:,i) = GenericINS.SensorFusionNoBiasEstimation.SensorModelVelocity(state, obj.gyrRaw, obj.posIMUBody2Sensor, obj.dcmIMUBody2Sensor, posBody2Sensor, dcmBody2Sensor);
                    end
            end

            % Calculate weighted mean of sigma-points
            ym = obj.w0 * Y(:,1) + obj.wi * sum(Y(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP), 2);

            % Calculate dY = Y - ym
            dY = (Y - repmat(ym, 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP));

            % Calculate the innovation
            innovation = measurement - ym;

            % QR decomposition of sigma points: additive SRSSUKF
            sqrtR = diag(stdMeasurement);
            [~, Sy_UPPER] = qr([(obj.srwi * dY(:,2:GenericINS.SensorFusionNoBiasEstimation.NUM_SP)) sqrtR]', 0);

            % Cholesky update or downdate (w0 is guaranteed to be positive -> update)
            Sy_UPPER = cholupdate(Sy_UPPER, obj.srw0 * dY(:,1), '+');
            Sy = Sy_UPPER';

            % Calculate covariance matrix Pxy
            Pxy = obj.w0 * obj.dX(:,1) * dY(:,1)' + obj.wi * obj.dX(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP) * dY(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP)';

            % Calculate the Kalman gain matrix
            K = (Pxy / Sy') / Sy;

            % Get state update dx = K * innovation
            dx = K * innovation;

            % Update state estimation x = x + dx taking angles and quaternions into account
            obj.x(1:6) = obj.x(1:6) + dx(1:6);
            obj.x(2) = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(obj.x(2));
            obj.x(7:10) = GenericINS.SensorFusionNoBiasEstimation.Qdot(GenericINS.SensorFusionNoBiasEstimation.OV2Q(dx(7:9)), obj.x(7:10));

            % Update sqrt of covariance using cholesky downdate. Because MATLABs cholesky update uses the UPPER triangle we have to transpose Sx.
            Sx_UPPER = obj.S(int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS,int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS)';
            U = K * Sy;
            for j = 1:yDim
                [Sx_UPPER,~] = cholupdate(Sx_UPPER, U(:,j), '-');
            end

            % Convert from UPPER cholesky factor to LOWER cholesky factor
            obj.S(int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS,int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS) = Sx_UPPER';
        end
        function UpdateSpeedOverGround(obj, measurement, stdMeasurement, posBody2Sensor)
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
            Y = zeros(1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP);
            for i = int32(1):GenericINS.SensorFusionNoBiasEstimation.NUM_SP
                state = obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i);
                Y(:,i) = GenericINS.SensorFusionNoBiasEstimation.SensorModelSpeedOverGround(state, obj.gyrRaw, obj.posIMUBody2Sensor, obj.dcmIMUBody2Sensor, posBody2Sensor);
            end

            % Calculate weighted mean of sigma-points
            ym = obj.w0 * Y(:,1) + obj.wi * sum(Y(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP), 2);

            % Calculate dY = Y - ym
            dY = (Y - repmat(ym, 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP));

            % Calculate the innovation
            innovation = measurement - ym;

            % QR decomposition of sigma points: additive SRSSUKF
            sqrtR = diag(stdMeasurement);
            [~, Sy_UPPER] = qr([(obj.srwi * dY(:,2:GenericINS.SensorFusionNoBiasEstimation.NUM_SP)) sqrtR]', 0);

            % Cholesky update or downdate (w0 is guaranteed to be positive -> update)
            Sy_UPPER = cholupdate(Sy_UPPER, obj.srw0 * dY(:,1), '+');
            Sy = Sy_UPPER';

            % Calculate covariance matrix Pxy
            Pxy = obj.w0 * obj.dX(:,1) * dY(:,1)' + obj.wi * obj.dX(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP) * dY(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP)';

            % Calculate the Kalman gain matrix
            K = (Pxy / Sy') / Sy;

            % Get state update dx = K * innovation
            dx = K * innovation;

            % Update state estimation x = x + dx taking angles and quaternions into account
            obj.x(1:6) = obj.x(1:6) + dx(1:6);
            obj.x(2) = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(obj.x(2));
            obj.x(7:10) = GenericINS.SensorFusionNoBiasEstimation.Qdot(GenericINS.SensorFusionNoBiasEstimation.OV2Q(dx(7:9)), obj.x(7:10));

            % Update sqrt of covariance using cholesky downdate. Because MATLABs cholesky update uses the UPPER triangle we have to transpose Sx.
            Sx_UPPER = obj.S(int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS,int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS)';
            U = K * Sy;
            [Sx_UPPER,~] = cholupdate(Sx_UPPER, U(:,1), '-');

            % Convert from UPPER cholesky factor to LOWER cholesky factor
            obj.S(int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS,int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS) = Sx_UPPER';
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
            Y = zeros(yDim, GenericINS.SensorFusionNoBiasEstimation.NUM_SP);
            switch(yDim)
                case 1
                    for i = int32(1):GenericINS.SensorFusionNoBiasEstimation.NUM_SP
                        state = obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i);
                        y = GenericINS.SensorFusionNoBiasEstimation.SensorModelOrientation(state, dcmBody2Sensor);
                        Y(:,i) = y(3);
                    end
                case 2
                    for i = int32(1):GenericINS.SensorFusionNoBiasEstimation.NUM_SP
                        state = obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i);
                        y = GenericINS.SensorFusionNoBiasEstimation.SensorModelOrientation(state, dcmBody2Sensor);
                        Y(:,i) = y(1:2);
                    end
                case 3
                    for i = int32(1):GenericINS.SensorFusionNoBiasEstimation.NUM_SP
                        state = obj.Xi(1:GenericINS.SensorFusionNoBiasEstimation.DIM_X,i);
                        Y(:,i) = GenericINS.SensorFusionNoBiasEstimation.SensorModelOrientation(state, dcmBody2Sensor);
                    end
            end

            % Get pivot angle from 0-th Y sigma-point and remove this angle from all sigma-points
            pivotAngle = Y(:,1);
            Y = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(Y - repmat(pivotAngle, 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP));

            % Calculate weighted mean of sigma-points
            ym = obj.w0 * Y(:,1) + obj.wi * sum(Y(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP), 2);

            % Calculate dY = Y - ym
            dY = (Y - repmat(ym, 1, GenericINS.SensorFusionNoBiasEstimation.NUM_SP));

            % Add pivot angle to mean value
            ym = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(ym + pivotAngle);

            % Calculate the innovation
            innovation = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(measurement - ym);

            % QR decomposition of sigma points: additive SRSSUKF
            sqrtR = diag(stdMeasurement);
            [~, Sy_UPPER] = qr([(obj.srwi * dY(:,2:GenericINS.SensorFusionNoBiasEstimation.NUM_SP)) sqrtR]', 0);

            % Cholesky update or downdate (w0 is guaranteed to be positive -> update)
            Sy_UPPER = cholupdate(Sy_UPPER, obj.srw0 * dY(:,1), '+');
            Sy = Sy_UPPER';

            % Calculate covariance matrix Pxy
            Pxy = obj.w0 * obj.dX(:,1) * dY(:,1)' + obj.wi * obj.dX(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP) * dY(:,int32(2):GenericINS.SensorFusionNoBiasEstimation.NUM_SP)';

            % Calculate the Kalman gain matrix
            K = (Pxy / Sy') / Sy;

            % Get state update dx = K * innovation
            dx = K * innovation;

            % Update state estimation x = x + dx taking angles and quaternions into account
            obj.x(1:6) = obj.x(1:6) + dx(1:6);
            obj.x(2) = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(obj.x(2));
            obj.x(7:10) = GenericINS.SensorFusionNoBiasEstimation.Qdot(GenericINS.SensorFusionNoBiasEstimation.OV2Q(dx(7:9)), obj.x(7:10));

            % Update sqrt of covariance using cholesky downdate. Because MATLABs cholesky update uses the UPPER triangle we have to transpose Sx.
            Sx_UPPER = obj.S(int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS,int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS)';
            U = K * Sy;
            for j = 1:yDim
                [Sx_UPPER,~] = cholupdate(Sx_UPPER, U(:,j), '-');
            end

            % Convert from UPPER cholesky factor to LOWER cholesky factor
            obj.S(int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS,int32(1):GenericINS.SensorFusionNoBiasEstimation.DIM_XS) = Sx_UPPER';
        end
    end
    methods(Static,Access=private)
        function y = Normalize(u)
            %GenericINS.SensorFusionNoBiasEstimation.Normalize Safe normalization.
            % 
            % PARAMETERS
            % u ... Input vector. Must contain at least one element!
            % 
            % RETURN
            % y ... Normalized output vector. If input vector is close to zero, a default vector will be returned.
            len = norm(u);
            if(len < 1e-12)
                u = zeros(size(u));
                u(1) = 1.0;
                len = 1.0;
            end
            y = (1.0 / len) * u;
        end
        function y = SymmetricalAngle(x)
            %GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle Convert a given angle x [rad] to an output angle y [rad] with y being in range [-pi, +pi).
            % 
            % PARAMETERS
            % x ... Input angle in radians, either scalar or n-dimensional.
            % 
            % RETURN
            % y ... Output angle in radians being in range [-pi, pi).
            x = x - 6.28318530717959 * fix(x * 0.159154943091895);
            y = x + 6.28318530717959 * double(x < -3.14159265358979) - 6.28318530717959 * double(x >= 3.14159265358979);
        end
        function Q = OV2Q(OV)
            %GenericINS.SensorFusionNoBiasEstimation.OV2Q Convert an orientation vector to a unit quaternion.
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
            %GenericINS.SensorFusionNoBiasEstimation.Q2OV Convert a unit quaternion to an orientation vector. The input quaternion will be normalized.
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
            %GenericINS.SensorFusionNoBiasEstimation.Qdot Calculate the quaternion multiplication for two unit quaternions Q and r. The output quaternion will be a unit quaternion.
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
            %GenericINS.SensorFusionNoBiasEstimation.Cb2n Calculate the rotation matrix for a given unit quaternion.
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
            %GenericINS.SensorFusionNoBiasEstimation.EulerZYX Calculate the corresponding euler angles (zyx-convention) for a given rotation matrix (b-frame to n-frame).
            % 
            % PARAMETERS
            % R ... 3x3 Rotation matrix (b-frame to n-frame).
            % 
            % RETURN
            % roll  ... Resulting roll angle in radians.
            % pitch ... Resulting pitch angle in radians.
            % yaw   ... Resulting yaw angle in radians.
            R = reshape(R, [3 3]);
            roll = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(atan2(R(3,2), R(3,3)));
            pitch = -real(asin(min(max(R(3,1),-1.0),1.0)));
            yaw = GenericINS.SensorFusionNoBiasEstimation.SymmetricalAngle(atan2(R(2,1), R(1,1)));
        end
        function [Rn, Re, R0] = WGS84(latitude)
            %GenericINS.SensorFusionNoBiasEstimation.WGS84 Get information about the WGS84 earth reference model for a given latitude in radians.
            % 
            % PARAMETERS
            % latitude ... Latitude in radians.
            % 
            % RETURN
            % Rn    ... Northern earth radius [m].
            % Re    ... Eastern earth radius [m].
            % R0    ... Mean earth radius [m].
            a = 6378137.0;
            ee = 0.00669437999014132;
            s = sin(latitude);
            invRoot = 1.0 / sqrt(1 - ee * s * s);
            Re = a * invRoot;
            Rn = Re * (1-ee) * (invRoot*invRoot);
            R0 = sqrt(Re * Rn);
        end
        function gravity = GravityWGS84(latitude, longitude, altitude)
            %GenericINS.SensorFusionNoBiasEstimation.GravityWGS84 Calculate the gravity according to the WGS84 reference model. This implementation
            % is based on MATLAB's gravitywgs84 function using the close approximation method, e.g. gravitywgs84(h,lat,lon,'CloseApprox',[false false false 0])
            % including atmosphere and centrifugal effects and excluding the presence of a precessing reference frame.
            % 
            % PARAMETERS
            % latitude   ... [double x 1] Latitude in radians.
            % longitude  ... [double x 1] Longitude in radians.
            % altitude   ... [double x 1] Altitude in meters, positive upwards.
            % 
            % RETURN
            % gravity    ... [double x 1] Gravity in m/s^2
            % 
            % DETAILS
            % Use of the WGS84 Close Approximation model should be limited to a geodetic height of 20,000.0 meters (approximately 65,620.0 feet).
            % Below this height, it gives results with submicrogal precision (see help of gravitywgs84).
            assert(isscalar(latitude) && isscalar(longitude) && isscalar(altitude), 'Inputs must be scalar!');

            % Some constants
            E        = 5.2185400842339e+5;  % = WGS->E;
            E2       = E*E;                 % = WGS84->E * WGS84->E;
            GM       = 3986004.418e+8;      % = WGS84->GM_default;
            a        = 6378137.0;           % = WGS->a;
            b        = 6356752.3142;        % = WGS->b;
            e2       = 6.69437999014e-3;    % = WGS->e2;
            b_over_a = 0.996647189335;      % = WGS->b_over_a;
            omega    = 7.292115e-5;         % = WGS->omega_default;

            % Precalculation of sin/cos
            sinphi = sin(latitude);
            cosphi = cos(latitude);
            sinlambda = sin(longitude);
            coslambda = cos(longitude);
            sin2phi = sinphi * sinphi;

            %/* Radius of Curvature in prime vertical (N) /eq. 4-15/ */
            N = (a) / (sqrt(1.0 - (e2) * sin2phi));

            %/* Calculate rectangular coordinates /eq. 4-14/ */
            x_rec = (N + altitude) * cosphi * coslambda;
            y_rec = (N + altitude) * cosphi * sinlambda;
            z_rec = ((b_over_a) * (b_over_a) * N + altitude) * sinphi;

            %/* Calculate various parameters */
            D = x_rec * x_rec + y_rec * y_rec + z_rec * z_rec - E2;
            u2 = 0.5 * D * (1.0 + sqrt(1.0 + 4.0 * E2 * z_rec * z_rec / (D * D)));
            u2E2 = u2 + E2;

            %/* /eq. 4-8/ */
            u = sqrt(u2);

            %/* /eq. 4-9/ */
            beta = atan(z_rec * sqrt(u2E2) / (u * sqrt(x_rec * x_rec + y_rec * y_rec)));

            %/* generate common sines and cosines */
            sinbeta = sin(beta);
            cosbeta = cos(beta);
            sin2beta = sinbeta * sinbeta;
            cos2beta = cosbeta * cosbeta;

            %/* /eq. 4-10/ */
            w = sqrt((u2 + E2 * sin2beta) / (u2E2));

            %/* /eq. 4-11/ */
            q = 0.5 * ((1.0 + 3.0 * u2 / (E2)) * atan((E) / u) - 3.0 * u / (E));

            %/* /eq. 4-12/ */
            qo = 0.5 * ((1.0 + 3.0 * (b) * (b) / (E2)) * atan((E) / (b)) - 3.0 * (b) / (E));

            %/* /eq. 4-13/ */
            q_prime = 3.0 * ((1.0 + u2 / (E2)) * (1.0 - (u / (E)) * atan((E) / u))) - 1.0;

            %/* Use Centrifugal Force */
            cf_u = u * cos2beta * omega * omega / w;
            cf_beta = sqrt(u2E2) * cosbeta * sinbeta * omega * omega / w;

            %/* /eq. 4-5/ */
            gamma_u = -(GM / u2E2 + omega * omega * (a) * (a) * (E) * q_prime * (0.5 * sin2beta - 1.0 / 6.0) / (u2E2 * qo)) / w + cf_u;

            %/* /eq. 4-6/ */
            gamma_beta = omega * omega * (a) * (a) * q * sinbeta * cosbeta / (sqrt(u2E2) * w * qo) - cf_beta;

            % Resulting gravity
            gravity = sqrt(gamma_u*gamma_u + gamma_beta*gamma_beta);
        end
        function [lat, lon] = LatLon(phi, lambda)
            %GenericINS.SensorFusionNoBiasEstimation.LatLon Convert phi [rad] and lambda [rad] to latitude [rad] and longitude [rad] with latitude being in range [-pi/2, +pi/2] and longitude being in range [-pi; +pi).
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
        function [lat, lon, alt] = NED2LLA(N, E, D, originLat, originLon, originAlt)
            %GenericINS.SensorFusionNoBiasEstimation.NED2LLA Convert local NED frame coordinates to LLA coordinates (WGS84).
            % 
            % PARAMETERS
            % N         ... North value of the NED position (m).
            % E         ... East value of the NED position (m).
            % D         ... Down value of the NED position (m).
            % originLat ... Latitude (rad) that indicates the origin of the local NED frame.
            % originLon ... Longitude (rad) that indicates the origin of the local NED frame.
            % originAlt ... Altitude (m, positive upwards) that indicates the origin of the local NED frame.
            % 
            % RETURN
            % lat ... Latitude (rad) of converted position.
            % lon ... Longitude (rad) of converted position.
            % alt ... Altitude (m, positive upwards) of converted position.

            % Constants and spheroid properties
            a = 6378137.0;
            f = 1.0 / 298.257223563;
            b = (1.0 - f) * a;       % Semiminor axis
            e2 = f * (2.0 - f);      % Square of (first) eccentricity
            ep2 = e2 / (1.0 - e2);   % Square of second eccentricity

            % ECEF: Computation of (x,y,z) = (x0,y0,z0) + (dx,dy,dz)
            slat = sin(originLat);
            clat = cos(originLat);
            slon = sin(originLon);
            clon = cos(originLon);
            e2 = f * (2.0 - f);
            Nval  = a ./ sqrt(1.0 - e2 * slat.^2);
            rho = (Nval + originAlt) .* clat;
            z0 = (Nval*(1.0 - e2) + originAlt) .* slat;
            x0 = rho .* clon;
            y0 = rho .* slon;
            t = clat .* (-D) - slat .* N;
            dz = slat .* (-D) + clat .* N;
            dx = clon .* t - slon .* E;
            dy = slon .* t + clon .* E;
            x = x0 + dx;
            y = y0 + dy;
            z = z0 + dz;
            lon = atan2(y,x);

            % Bowring's formula for initial parametric (beta) and geodetic
            rho = hypot(x,y);
            beta = atan2(z, (1.0 - f) * rho);
            lat = atan2(z   + b * ep2 * sin(beta).^3,...
            rho - a * e2  * cos(beta).^3);

            % Fixed-point iteration with Bowring's formula (typically converges within two or three iterations)
            betaNew = atan2((1.0 - f)*sin(lat), cos(lat));
            count = 0;
            while(any(beta(:) ~= betaNew(:)) && count < 5)
                beta = betaNew;
                lat = atan2(z + b * ep2 * sin(beta).^3, rho - a * e2 * cos(beta).^3);
                betaNew = atan2((1.0 - f)*sin(lat), cos(lat));
                count = count + 1;
            end

            % Ellipsoidal height from final value for latitude
            slat = sin(lat);
            Nval = a ./ sqrt(1.0 - e2 * slat.^2);
            alt = rho .* cos(lat) + (z + e2 * Nval .* slat) .* slat - Nval;
        end
        function xOut = ProcessModelEuler(x, w, u, Ts, dcmIMUBody2Sensor)
            % Output size
            xOut = x;

            % WGS84 properties
            [Rn, Re, ~] = GenericINS.SensorFusionNoBiasEstimation.WGS84(x(1));
            localGravity = GenericINS.SensorFusionNoBiasEstimation.GravityWGS84(x(1), x(2), x(3));

            % Inputs: transform noisy acceleration and angular rate to b-frame
            f_ib = dcmIMUBody2Sensor' * (u(1:3) + w(1:3));
            w_ib = dcmIMUBody2Sensor' * (u(4:6) + w(4:6));

            % Ensure Current attitude
            x(7:10) = GenericINS.SensorFusionNoBiasEstimation.Normalize(x(7:10));
            DCM_b2n = GenericINS.SensorFusionNoBiasEstimation.Cb2n(x(7:10));
            DCM_n2b = DCM_b2n';

            % Predict position
            xOut(1) = x(1) + Ts * (x(4) / (Rn + x(3)));
            xOut(2) = x(2) + Ts * (x(5) / ((Re + x(3)) * cos(x(1))));
            xOut(3) = x(3) - Ts * (x(6));

            % lat/lon range conversion
            [lat, lon] = GenericINS.SensorFusionNoBiasEstimation.LatLon(xOut(1), xOut(2));
            xOut(1) = lat;
            xOut(2) = lon;

            % Predict velocity
            w_ie = [GenericINS.SensorFusionNoBiasEstimation.OMEGA_EARTH * cos(x(1)); 0; -GenericINS.SensorFusionNoBiasEstimation.OMEGA_EARTH * sin(x(1))];
            w_en = [(x(5) / ((Re + x(3)))); -(x(4) / (Rn + x(3))); -(x(5) / (Re + x(3)))*tan(x(1))];
            coriolis = cross(2.0 * w_ie + w_en, x(4:6));
            xOut(4:6) = x(4:6) + Ts * (DCM_b2n * f_ib - coriolis + [0.0; 0.0; localGravity]);

            % Predict attitude
            w_nb = w_ib - DCM_n2b * (w_ie + w_en);
            Omega = [0 -w_nb(1) -w_nb(2) -w_nb(3); w_nb(1) 0 w_nb(3) -w_nb(2); w_nb(2) -w_nb(3) 0 w_nb(1); w_nb(3) w_nb(2) -w_nb(1) 0];
            xOut(7:10) = (eye(4)*(1 - Ts*Ts*(w_nb(1)*w_nb(1) + w_nb(2)*w_nb(2) + w_nb(3)*w_nb(3))/8) + 0.5*Ts*Omega) * x(7:10);
            xOut(7:10) = GenericINS.SensorFusionNoBiasEstimation.Normalize(xOut(7:10));
        end
        function y = SensorModelPosition(x, posIMUBody2Sensor, posPOSBody2Sensor)
            % Vector from IMU to GNSS in NED-frame
            q = GenericINS.SensorFusionNoBiasEstimation.Normalize(x(7:10));
            r_NED = GenericINS.SensorFusionNoBiasEstimation.Cb2n(q) * (posPOSBody2Sensor - posIMUBody2Sensor);

            % Transform from NED-frame to geographic coordinates (WGS84)
            [lat, lon, alt] = GenericINS.SensorFusionNoBiasEstimation.NED2LLA(r_NED(1),r_NED(2),r_NED(3),x(1),x(2),x(3));
            y = [lat; lon; alt];
        end
        function y = SensorModelVelocity(x, gyrRaw, posIMUBody2Sensor, dcmIMUBody2Sensor, posBody2Sensor, dcmBody2Sensor)
            % Direction cosine matrices from current quaternion
            DCM_b2n = GenericINS.SensorFusionNoBiasEstimation.Cb2n(GenericINS.SensorFusionNoBiasEstimation.Normalize(x(7:10)));
            DCM_n2b = DCM_b2n';

            % Angular rate of body with respect to the earth
            w_ie = GenericINS.SensorFusionNoBiasEstimation.OMEGA_EARTH * [cos(x(1)); 0; -sin(x(1))];
            w_eb = dcmIMUBody2Sensor' * gyrRaw - DCM_n2b * w_ie;

            % Additional velocity due to angular speed + sensor alignment in b-frame
            v_b = cross(w_eb, posBody2Sensor - posIMUBody2Sensor);

            % Transform additional b-frame velocity to n-frame and add current prediction of v_eb (n-frame)
            v_n = x(4:6) + DCM_b2n * v_b;

            % Finally, transform velocity to sensor frame of the DVL
            y = dcmBody2Sensor * DCM_n2b * v_n;
        end
        function y = SensorModelSpeedOverGround(x, gyrRaw, posIMUBody2Sensor, dcmIMUBody2Sensor, posBody2Sensor)
            % Direction cosine matrices from current quaternion
            DCM_b2n = GenericINS.SensorFusionNoBiasEstimation.Cb2n(GenericINS.SensorFusionNoBiasEstimation.Normalize(x(7:10)));
            DCM_n2b = DCM_b2n';

            % Angular rate of body with respect to the earth
            w_ie = GenericINS.SensorFusionNoBiasEstimation.OMEGA_EARTH * [cos(x(1)); 0; -sin(x(1))];
            w_eb = dcmIMUBody2Sensor' * gyrRaw - DCM_n2b * w_ie;

            % Additional velocity due to angular speed + sensor alignment in b-frame
            v_b = cross(w_eb, posBody2Sensor - posIMUBody2Sensor);

            % Transform additional b-frame velocity to n-frame and add current prediction of v_eb (n-frame)
            v_n = x(4:6) + DCM_b2n * v_b;

            % Speed over ground
            y = sqrt(v_n(1)*v_n(1) + v_n(2)*v_n(2));
        end
        function y = SensorModelOrientation(x, dcmBody2Sensor)
            % Get attitude from state vector
            q = GenericINS.SensorFusionNoBiasEstimation.Normalize(x(7:10));

            % Transform to roll, pitch, yaw
            DCM_S2N = GenericINS.SensorFusionNoBiasEstimation.Cb2n(q) * dcmBody2Sensor';
            [roll,pitch,yaw] = GenericINS.SensorFusionNoBiasEstimation.EulerZYX(DCM_S2N);

            % Output
            y = [roll; pitch; yaw];
        end
    end
    properties(Constant,Access=private)
        % Fixed dimension for generic INS problem: DO NOT CHANGE!
        DIM_X  = int32(10);                                                                                       % Dimension of state vector (x).
        DIM_XS = GenericINS.SensorFusionNoBiasEstimation.DIM_X - int32(1);                                        % Dimension of state vector (x) with respect to unvertainty S.
        DIM_W  = int32(6);                                                                                        % Dimension of process noise (w).
        DIM_L  = GenericINS.SensorFusionNoBiasEstimation.DIM_X + GenericINS.SensorFusionNoBiasEstimation.DIM_W;   % Dimension of augmented state vector.
        DIM_LS = GenericINS.SensorFusionNoBiasEstimation.DIM_XS + GenericINS.SensorFusionNoBiasEstimation.DIM_W;  % Dimension of augmented state vector with respect to uncertainty S.
        NUM_SP = GenericINS.SensorFusionNoBiasEstimation.DIM_LS + int32(2);                                       % Number of sigma-points.

        % WGS84 Properties
        OMEGA_EARTH = 7.292115e-5;                                                                                % Angular rate [rad/s] of the earth w.r.t. the inertial frame.
    end
    properties(Access=private)
        initialized;       % True if filter is initialized, false otherwise.
        firstPrediction;   % True if filter performed the first prediction after the initialization.
        Z;                 % Spherical simplex sigma point matrix.
        x;                 % Augmented state vector (L x 1).
        S;                 % Augmented square-root covariance matrix.
        Xi;                % Matrix of sigma points (L x numSP).
        dX;                % Xi - x (DIM_XS x numSP).
        spU2D;             % True if sigma points are up to date, false otherwise.
        gyrRaw;            % Raw gyroscope data from IMU (sensor frame).
        dcmIMUBody2Sensor; % Direction cosine matrix for rotation from body frame to sensor frame.
        posIMUBody2Sensor; % Position of IMU sensor frame in body frame (b-frame origin to s-frame origin in b-frame coords).
        w0;                % Scalar weighting factor for 0-th sigma point, range: [0;1).
        wi;                % Weighting factor for other sigma points (calculated from w0)
        srw0;              % Square root of w0.
        srwi;              % Square root of wi.
    end
end


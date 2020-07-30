% SRSSUKF.m
% 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Version     Author                 Changes
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 20181008    Robert Damerius        Initial release.
% 20200730    Robert Damerius        Increased performance of SymmetricalAngle() function.
% 
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% 
% 
%SRSSUKF Square-root spherical simplex unscented kalman filter
% This class can be used for state estimation using nonlinear process and/or measurement models. Use the constructor to setup
% a square-root SSUKF (See SRSSUKF.SRSSUKF). Once you created the object, call the SRSSUKF.Initialize member function to set an initial
% state and square-root covariance. Then you can use the SRSSUKF.Predict member function for time-update and the
% SRSSUKF.Update member function for measurement updates. It is possible to call the SRSSUKF.Update member function several times
% one after another (sequential sensor method). The state can contain one or more unit quaternions and/or angles. An observation
% can contain one or more angles.
% 
% EXAMPLE:
% % Create filter object
% w0   = 0.5;    % weight parameter
% xDim = 2;      % state dimension
% wDim = 2;      % process noise dimension
% idxA = [];     % no angles in x
% idxQ = [];     % no quaternions in x
% yDim = {1, 1}; % Two sensors yDim1 = 1, yDim2 = 1
% vDim = {1, 1}; % Two sensors vDim1 = 1, vDim2 = 1
% filter = SRSSUKF(w0, xDim, wDim, idxA, idxQ, yDim, vDim);
% 
% % Initialize filter
% x0 = [0; 0];
% P  = eye(2);
% Q  = eye(2);
% R  = eye(2);
% Sx = chol(P,'lower');
% Sw = chol(Q,'lower');
% Sv = chol(R,'lower');
% S0 = blkdiag(Sx, Sw, Sv);
% filter.Initialize(x0, S0);
% 
% % Predict
% Ts = 0.01;
% u  = 0;
% funcProc = @(x,w,u,Ts,optArg)([x(1) + Ts*x(2); x(2) + Ts*u]);
% filter.Predict(funcProc, Ts, u, [], []);
% 
% % Update using both sensor measurements
% y1 = 0;
% y2 = 0;
% funcObsv1 = @(x,v,optArg)(x(1));
% funcObsv2 = @(x,v,optArg)(x(2));
% filter.Update(funcObsv1, 1, y1, [], [], []);
% filter.Update(funcObsv2, 2, y2, [], [], []);

classdef SRSSUKF < handle
    methods
        function obj = SRSSUKF(w0, xDim, wDim, xIdxAngle, xIdxQuaternion, cyDim, cvDim)
            %SRSSUKF.SRSSUKF Create a square-root spherical simplex unscented kalman filter.
            % 
            % PARAMETERS
            % w0             ... Scalar weighting factor for 0-th sigma-point, range: [0;1).
            % xDim           ... Dimension of the actual state vector x.
            % wDim           ... Dimension of the process noise vector w (can be zero for additive SRSSUKF).
            % xIdxAngle      ... Indices of x which denote angles components (if state contains no angles use []).
            % xIdxQuaternion ... Indices of x which denote quaternion components q0 (if state contains no quaternions use []).
            %                    A quaternion is given by [q0 q1 q2 q3]'. However, only specify index of q0!
            % cyDim          ... Cell array containing y-dimensions for all observations.
            % cvDim          ... Cell array containing v-dimensions for all measurement noise vectors.

            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            % Check for valid inputs
            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            % Check size and value of weighting
            assert((1 == size(w0,1)) && (1 == size(w0,2)));
            assert((w0 >= 0.0) && (w0 < 1.0));

            % Check state dimension
            assert((1 == size(xDim,1)) && (1 == size(xDim,2)));
            xDim = int32(xDim);
            assert(xDim > 0);

            % Check process noise dimension
            if isempty(wDim)
                wDim = int32(0);
            else
                assert((1 == size(wDim,1)) && (1 == size(wDim,2)));
                wDim = int32(wDim);
                assert(wDim >= 0);
            end

            % Check angle indices for process model
            obj.idxAngle = int32([]);
            if ~isempty(xIdxAngle)
                obj.idxAngle = int32(xIdxAngle);
                num = max(size(obj.idxAngle));
                obj.idxAngle = reshape(obj.idxAngle, [num 1]);
                for n=1:num
                    assert((obj.idxAngle(n) > 0) && (obj.idxAngle(n) <= xDim));
                    for j=1:(n-1)
                        % Do not allow multiple indices with same value
                        assert(obj.idxAngle(n) ~= obj.idxAngle(j));
                    end
                end
            end

            % Check quaternion indices for process model
            obj.idxQuaternion = int32([]);
            if ~isempty(xIdxQuaternion)
                obj.idxQuaternion = int32(xIdxQuaternion);
                num = max(size(obj.idxQuaternion));
                obj.idxQuaternion = reshape(obj.idxQuaternion, [num 1]);
                for n=1:num
                    assert((obj.idxQuaternion(n) > 0) && (obj.idxQuaternion(n) < (xDim - 2)));
                    for j=1:(n-1)
                        % Do not allow overlapping quaternion indices
                        assert(abs(obj.idxQuaternion(n) - obj.idxQuaternion(j)) > 3);
                    end
                end
            end

            % Indices for angles and quaternions must not overlap
            for a=1:size(obj.idxAngle,1)
                for q=1:size(obj.idxQuaternion,1)
                    % Index of angle must be before quaternion or after quaternion
                    assert((obj.idxAngle(a) < obj.idxQuaternion(q)) || (obj.idxAngle(a) > (3 + obj.idxQuaternion(q))));
                end
            end

            % Check for valid observation models configuration
            assert(max(size(cyDim)) == max(size(cvDim)));
            obj.numObsv = int32(max(size(cyDim)));
            obj.infoObsv = int32(zeros(3,obj.numObsv));
            vDimAll = int32(0);
            for n=1:obj.numObsv
                % Check observation dimension
                assert((1 == size(cyDim{n},1)) && (1 == size(cyDim{n},2)));
                yDim = int32(cyDim{n});
                assert(yDim > 0);
                obj.infoObsv(1,n) = yDim;

                % Check observation noise dimension
                if isempty(cvDim{n})
                    vDim = int32(0);
                else
                    assert((1 == size(cvDim{n},1)) && (1 == size(cvDim{n},2)));
                    vDim = int32(cvDim{n});
                    assert(vDim >= 0);
                end
                obj.infoObsv(3,n) = vDim;

                % Set sigma-point index offset for current observation noise
                obj.infoObsv(2,n) = int32(1 + xDim + wDim + vDimAll);
                vDimAll = vDimAll + vDim;
            end

            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            % Set initial parameters
            % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            % Set dimensions
            obj.nx = xDim;
            obj.nxS = xDim;
            obj.nw = wDim;
            obj.L = int32(xDim + wDim + vDimAll);
            obj.LS = obj.L;
            if ~isempty(obj.idxQuaternion)
                numQuaternions = int32(size(obj.idxQuaternion,1));
                obj.LS = int32(obj.LS - numQuaternions);
                obj.nxS = int32(obj.nxS - numQuaternions);
            end
            obj.numSP = int32(obj.LS + 2);

            % Apply dimensions for state/covariance/sigma-points
            obj.x = zeros(obj.L, 1);
            obj.Xi = zeros(obj.L, obj.numSP);
            obj.S = zeros(obj.LS, obj.LS);
            obj.dX = zeros(obj.nxS, obj.numSP);

            % Calculate weights
            obj.w0 = w0;
            obj.wi = (1.0 - w0) / (double(obj.LS) + 1.0);
            obj.srw0 = sqrt(abs(obj.w0));
            obj.srwi = sqrt(obj.wi);

            % Initialize vector sequence for weighting matrix Z
            obj.Z = zeros(obj.LS, obj.numSP);
            obj.Z(1,3) = 1.0 / sqrt(2.0 * obj.wi);
            obj.Z(1,2) = -obj.Z(1,3);

            % Expand vector sequence
            for j = 2:obj.LS
                s = 1.0 / sqrt(double(j * (j + 1)) * obj.wi);
                obj.Z(j, j + 2) = double(j) * s;
                for k = 2:(j + 1)
                    obj.Z(j, k) = -s;
                end
            end

            % No sigma-points generated
            obj.spU2D = false;
        end
        function Initialize(obj, x0, S0)
            %SRSSUKF.Initialize Initialize the estimator using an initial state x0 and an initial square-root covariance S0.
            % 
            % PARAMETERS
            % x0 ... Nx1 initial state vector with N being the dimension of the non-augmented state vector x.
            % S0 ... MxM initial square-root covariance of the state where M depends on state augmentation and the number of used quaternions. S0 must be the
            %        LOWER cholesky factor!

            % Check dimensions
            assert((size(x0,1) == obj.nx) && (size(x0,2) == 1));
            assert((size(S0,1) == obj.LS) && (size(S0,2) == obj.LS));

            % Assign values
            obj.x(1:obj.L) = zeros(obj.L,1);
            obj.x(1:obj.nx) = x0;
            obj.S(1:obj.LS,1:obj.LS) = S0;

            % Sigma-points are no longer up-to-date
            obj.spU2D = false;
        end
        function state = Predict(obj, funcProc, Ts, u, sqrtQ, optArg)
            %SRSSUKF.Predict Perform a prediction (time-update) step.
            % 
            % PARAMETERS
            % funcProc ... The discrete-time (non)-linear process model with a function prototype: x = f(x, w, u, Ts, optArg). w will be a zero vector of
            %              size x when additive SRSSUKF is used.
            % Ts       ... Discrete sample time in seconds (is forwarded to user-defined funcProc).
            % u        ... Input vector (is forwarded to user-defined funcProc).
            % sqrtQ    ... Lower Cholesky factor of Q. If state is augmented with process noise w, this parameter will not be used.
            % optArg   ... Optional arguments, that are forwarded to user-defined funcProc.
            % 
            % RETURN
            % state    ... The current state estimation.

            % Generate sigma points
            obj.GenerateSigmaPoints();

            % Propagate through process model
            if(obj.nw)
                for i = 1:obj.numSP
                    obj.Xi(1:obj.nx,i) = funcProc(obj.Xi(1:obj.nx,i), obj.Xi((1+obj.nx):(obj.nx+obj.nw),i), u, Ts, optArg);
                end
            else
                for i = 1:obj.numSP
                    obj.Xi(1:obj.nx,i) = funcProc(obj.Xi(1:obj.nx,i), zeros(obj.nx,1), u, Ts, optArg);
                end
            end

            % Calculate dX = Xi - x
            obj.dXUpdate();
            state = obj.x(1:obj.nx);

            % Augmented state or additive SRSSUKF
            if obj.nw
                [~, Sx_UPPER] = qr((obj.srwi*obj.dX(:,2:obj.numSP))', 0);
            else
                assert((size(sqrtQ,1) == obj.nxS) && (size(sqrtQ,2) == obj.nxS));
                [~, Sx_UPPER] = qr([(obj.srwi*obj.dX(:,2:obj.numSP)) sqrtQ]', 0);
            end

            % Cholesky update or downdate (w0 is guaranteed to be positive)
            %if(obj.w0 < 0)
            %    [Sx_UPPER,~] = cholupdate(Sx_UPPER, obj.srw0*obj.dX(:,1), '-');
            %else
                Sx_UPPER = cholupdate(Sx_UPPER, obj.srw0*obj.dX(:,1), '+');
            %end
            obj.S(1:obj.nxS,1:obj.nxS) = Sx_UPPER';
        end
        function [state, innovation] = Update(obj, funcObsv, idx, y, idxAngle, sqrtR, optArg)
            %SRSSUKF.Update Perform a measurement-update step.
            % 
            % PARAMETERS
            % funcObsv   ... The discrete-time (non)-linear observation model with a function prototype: y = h(x, v, optArg). v will be a zero vector of size y
            %                if additive SRSSUKF is used.
            % idx        ... The index of the sensor. The dimensions of observation y and noise v are given by the cell arrays cyDim and cvDim that have been
            %                set by the constructor, respectively. idx is used to obtain yDim and vDim for the current sensor.
            % y          ... The noisy sensor measurement.
            % idxAngle   ... Indices of y which denote angles components (if observation contains no angles use []).
            % sqrtR      ... Lower Cholesky factor of R. If state is augmented with observation noise v, this parameter will not be used.
            % optArg     ... Optional arguments, that are forwarded to user-defined funcObsv.
            % 
            % RETURN
            % state      ... The current state estimation.
            % innovation ... The innovation (y - ym).

            % If sigma-points are not up-to-date: generate a new set of sigma-points, else use existing set.
            if ~obj.spU2D
                obj.GenerateSigmaPoints();
                obj.dXUpdate();
            end

            % Sigma-points are no longer up to date for next Update()-call
            obj.spU2D = false;

            % Check for correct index
            idx = int32(idx);
            assert(~isempty(idx));
            assert((idx > 0) && (idx <= obj.numObsv));
            yDim = obj.infoObsv(1,idx);
            vIndexOffset = obj.infoObsv(2,idx);
            vDim = obj.infoObsv(3,idx);

            % Check for correct angle indices
            idxAngle = int32(idxAngle);
            nia = max(size(idxAngle));
            if ~isempty(idxAngle)
                assert((1 == size(idxAngle,1)) || (1 == size(idxAngle,2)));
            end
            for i=1:nia
                assert((idxAngle(i) > 0) && (idxAngle(i) <= yDim));
            end

            % Check for correct measurement vector
            assert(size(y,1) == yDim);

            % Propagate sigma-points through observation model
            Y = zeros(yDim, obj.numSP);
            if(vDim)
                for i = 1:obj.numSP
                    Y(:,i) = funcObsv(obj.Xi(1:obj.nx,i), obj.Xi(vIndexOffset:(vIndexOffset+vDim-1),i), optArg);
                end
            else
                for i = 1:obj.numSP
                    Y(:,i) = funcObsv(obj.Xi(1:obj.nx,i), zeros(yDim,1), optArg);
                end
            end

            % Get pivot angle from 0-th Y sigma-point and remove this angle from all sigma-points
            pivotAngle = Y(idxAngle,1);
            if ~isempty(idxAngle)
                Y(idxAngle,:) = SRSSUKF.SymmetricalAngle(Y(idxAngle,:) - repmat(pivotAngle, 1, obj.numSP));
            end

            % Calculate weighted mean of sigma-points
            ym = obj.w0 * Y(:,1) + obj.wi * sum(Y(:,2:obj.numSP), 2);

            % Calculate dY = Y - ym
            dY = (Y - repmat(ym, 1, obj.numSP));

            % Add pivot angle to mean value
            if ~isempty(idxAngle)
                ym(idxAngle) = SRSSUKF.SymmetricalAngle(ym(idxAngle) + pivotAngle);
            end

            % Calculate the innovation
            innovation = y - ym;
            if ~isempty(idxAngle)
                innovation(idxAngle) = SRSSUKF.SymmetricalAngle(innovation(idxAngle));
            end

            % QR decomposition of sigma points: augmented state or additive SRSSUKF
            if vDim
                [~, Sy_UPPER] = qr((obj.srwi*dY(:,2:obj.numSP))', 0);
            else
                assert((size(sqrtR,1) == yDim) && (size(sqrtR,2) == yDim));
                [~, Sy_UPPER] = qr([(obj.srwi*dY(:,2:obj.numSP)) sqrtR]', 0);
            end

            % Cholesky update or downdate (w0 is guaranteed to be positive)
            %if(obj.w0 < 0)
            %    [Sy_UPPER,~] = cholupdate(Sy_UPPER, obj.srw0*dY(:,1), '-');
            %else
                Sy_UPPER = cholupdate(Sy_UPPER, obj.srw0*dY(:,1), '+');
            %end
            Sy = Sy_UPPER';

            % Calculate covariance matrix Pxy
            Pxy = obj.w0*obj.dX(:,1)*dY(:,1)' + obj.wi*obj.dX(:,2:obj.numSP)*dY(:,2:obj.numSP)';

            % Calculate the Kalman gain matrix
            K = (Pxy / Sy') / Sy;

            % Get state update dx = K * innovation
            dx = K * innovation;

            % Update state estimation x = x + dx taking angles and quaternions into account
            rowdx = 1;
            rowx = 1;
            for n=1:obj.nx
                isQuaternion = false;
                isAngle = false;
                for i = 1:size(obj.idxQuaternion,1)
                    isQuaternion = (obj.idxQuaternion(i) == rowx);
                    if isQuaternion, break; end
                end
                for i = 1:size(obj.idxAngle,1)
                    isAngle = (obj.idxAngle(i) == rowx);
                    if isAngle, break; end
                end
                if isQuaternion
                    % Convert orientation vector components of dx to quaternion
                    qdx = SRSSUKF.OV2Q(dx(rowdx:(rowdx+2)));

                    % Update quaternion state using quat. product
                    obj.x(rowx:(rowx+3)) = SRSSUKF.Qdot(qdx, obj.x(rowx:(rowx+3)));

                    % Update indices
                    rowx = rowx + 4;
                    rowdx = rowdx + 3;
                else
                    % Straight forward for usual state
                    obj.x(rowx) = obj.x(rowx) + dx(rowdx);
                    if isAngle
                        obj.x(rowx) = SRSSUKF.SymmetricalAngle(obj.x(rowx));
                    end

                    % Update indices
                    rowx = rowx + 1;
                    rowdx = rowdx + 1;
                end
                if(rowx > obj.nx), break; end
            end
            state = obj.x(1:obj.nx);

            % Update sqrt of covariance using cholesky update. Because MATLABs cholesky update uses the UPPER triangle we have to transpose Sx.
            Sx_UPPER = obj.S(1:obj.nxS,1:obj.nxS)';
            U = K * Sy;
            for j = 1:yDim
                [Sx_UPPER,~] = cholupdate(Sx_UPPER, U(:,j), '-');
            end

            % Convert from UPPER cholesky factor to LOWER cholesky factor
            obj.S(1:obj.nxS,1:obj.nxS) = Sx_UPPER';
        end
        function state = GetState(obj)
            %SRSSUKF.GetState Get the current state estimation.
            % 
            % RETURN
            % state ... Nx1 vector of the current state estimation with N being the dimension of the non-augmented state.
            state = obj.x(1:obj.nx);
        end
        function Sx = GetSqrtCovariance(obj)
            %SRSSUKF.GetSqrtCovariance Get the square-root of the covariance matrix.
            % 
            % RETURN
            % Sx ... NxN square-root state covariance matrix with N being the dimension of the non-augmented state. Note that quaternion-components are replaced
            %        by corresponding orientation-vector components.
            Sx = obj.S(1:obj.nxS,1:obj.nxS);
        end
        function Px = GetCovariance(obj)
            %SRSSUKF.GetCovariance Get the covariance matrix.
            % 
            % RETURN
            % Px ... NxN state covariance matrix with N being the dimension of the non-augmented state. Note that quaternion-components are replaced by
            %        corresponding orientation-vector components.
            Px = obj.S(1:obj.nxS,1:obj.nxS) * obj.S(1:obj.nxS,1:obj.nxS)';
        end
        function SetSqrtQ(obj, sqrtQ)
            %SRSSUKF.SetSqrtQ Set square-root of process covariance Q.
            % 
            % PARAMETERS
            % sqrtQ ... wDim x wDim matrix, LOWER Cholesky factor of process covariance Q.

            % Check input dimension
            assert((size(sqrtQ,1) == obj.nw) && (size(sqrtQ,2) == obj.nw));

            % Set sqrt(Q)
            obj.S((obj.nxS+1):(obj.nxS+obj.nw),(obj.nxS+1):(obj.nxS+obj.nw)) = sqrtQ;
        end
        function SetSqrtR(obj, idx, sqrtR)
            %SRSSUKF.SetSqrtR Set square-root of observation covariance R.
            % 
            % PARAMETERS
            % idx   ... The index of the sensor. The dimensions of observation y and noise v are given by the cell arrays cyDim and cvDim that have been set by
            %           the constructor, respectively. idx is used to obtain vDim for the current sensor.
            % sqrtR ... vDim x vDim matrix, LOWER Cholesky factor of observation covariance R.

            % Check for correct index
            idx = int32(idx);
            assert((idx > 0) && (idx <= obj.numObsv));
            vIndexOffset = obj.infoObsv(2,idx);
            vDim = obj.infoObsv(3,idx);

            % Check input dimension
            assert((size(sqrtR,1) == vDim) && (size(sqrtR,2) == vDim));

            % vIndexOffset denotes the sigma-point offset (including quaternions). However, we need the uncertainty offset, that uses orientation vector
            % components:
            vIndexOffset = vIndexOffset + obj.nxS - obj.nx;

            % Set sqrt(R)
            obj.S(vIndexOffset:(vIndexOffset+vDim-1),vIndexOffset:(vIndexOffset+vDim-1)) = sqrtR;
        end
    end
    methods(Static)
        function y = SymmetricalAngle(x)
            %SRSSUKF.SymmetricalAngle Convert a given angle x [rad] to an output angle y [rad] with y being in range [-pi, +pi).
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
            %SRSSUKF.OV2Q Convert an orientation vector to a unit quaternion.
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
            %SRSSUKF.Q2OV Convert a unit quaternion to an orientation vector. The input quaternion will be normalized.
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
            %SRSSUKF.Qdot Calculate the quaternion multiplication for two unit quaternions Q and r. The output quaternion will be a unit quaternion.
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
    end
    methods(Access=private)
        function GenerateSigmaPoints(obj)
            % 0-th sigma point is equal to mean
            obj.Xi(:,1) = obj.x;

            % Perform multiplication
            SZ = obj.S * obj.Z(:,2:obj.numSP);

            % Convert orientation-vector components to quaternion-components
            SZq = zeros(obj.L, obj.numSP-1);
            rowSZ = 1;
            rowSZq = 1;
            for n = 1:obj.nx
                isQuaternion = false;
                for i = 1:size(obj.idxQuaternion,1)
                    isQuaternion = (obj.idxQuaternion(i) == rowSZq);
                    if isQuaternion, break; end
                end
                if isQuaternion
                    SZq(rowSZq:(rowSZq+3),:) = SRSSUKF.OV2Q(SZ(rowSZ:(rowSZ+2),:));
                    rowSZq = rowSZq + 4;
                    rowSZ = rowSZ + 3;
                else
                    SZq(rowSZq,:) = SZ(rowSZ,:);
                    rowSZq = rowSZq + 1;
                    rowSZ = rowSZ + 1;
                end
                if(rowSZq > obj.nx), break; end
            end
            SZq(rowSZq:end,:) = SZ(rowSZ:end,:);

            % Calculate sigma points Xi = x + SZ with special treatment for quaternion parts
            row = 1;
            for n = 1:obj.L
                isQuaternion = false;
                for i = 1:size(obj.idxQuaternion,1)
                    isQuaternion = (obj.idxQuaternion(i) == row);
                    if isQuaternion, break; end
                end
                if isQuaternion
                    obj.Xi(row:(row+3),2:end) = SRSSUKF.Qdot(SZq(row:(row+3),:),obj.x(row:(row+3)));
                    row = row + 4;
                else
                    obj.Xi(row, 2:end) = repmat(obj.x(row),1,(obj.numSP-1)) + SZq(row,:);
                    row = row + 1;
                end
                if (row > obj.L), break; end
            end
            obj.spU2D = true;
        end
        function dXUpdate(obj)
            % Get the pivot angle from 0-th sigma point and remove this angle from all sigma points
            pivotAngle = obj.Xi(obj.idxAngle,1);
            if ~isempty(obj.idxAngle)
                obj.Xi(obj.idxAngle,:) = SRSSUKF.SymmetricalAngle(obj.Xi(obj.idxAngle,:) - repmat(pivotAngle, 1, obj.numSP));
            end

            % Calculate weighted mean of sigma points (barycentric mean for quaternion part)
            obj.x(1:obj.nx) = obj.w0 * obj.Xi(1:obj.nx,1) + obj.wi * sum(obj.Xi(1:obj.nx,2:obj.numSP), 2);
            for i = 1:size(obj.idxQuaternion,1)
                obj.x(obj.idxQuaternion(i):(obj.idxQuaternion(i)+3)) = (1.0 / norm(obj.x(obj.idxQuaternion(i):(obj.idxQuaternion(i)+3)))) * obj.x(obj.idxQuaternion(i):(obj.idxQuaternion(i)+3));
            end

            % Calculate Xi - x
            rowXi = 1;
            rowdX = 1;
            for n = 1:obj.nx
                isQuaternion = false;
                isAngle = false;
                for i = 1:size(obj.idxQuaternion,1)
                    isQuaternion = (obj.idxQuaternion(i) == rowXi);
                    if isQuaternion, break; end
                end
                for i = 1:size(obj.idxAngle,1)
                    isAngle = (obj.idxAngle(i) == rowXi);
                    if isAngle, break; end
                end
                if isQuaternion
                    % Special treatment for quaternion part. Xi - x: We need the inverse quaternion because of -x.
                    q_inv = [obj.x(rowXi); -obj.x((rowXi+1):(rowXi+3))];

                    % Use quaternion multiplication for dX=Xi-x
                    dXq = SRSSUKF.Qdot(obj.Xi(rowXi:(rowXi+3),:), q_inv);

                    % Convert quaternion to orientation vector components
                    obj.dX(rowdX:(rowdX+2),:) = SRSSUKF.Q2OV(dXq);

                    % Update indices
                    rowXi = rowXi + 4;
                    rowdX = rowdX + 3;
                else
                    % Straight forward for usual state
                    obj.dX(rowdX,:) = obj.Xi(rowXi,:) - repmat(obj.x(rowXi),1,obj.numSP);
                    if isAngle
                        obj.dX(rowdX,:) = SRSSUKF.SymmetricalAngle(obj.dX(rowdX,:));
                    end

                    % Update indices
                    rowXi = rowXi + 1;
                    rowdX = rowdX + 1;
                end
                if(rowXi > obj.nx), break; end
            end

            % Add pivot angle to state (and also sigma-points for an upcomming measurement update)
            if ~isempty(obj.idxAngle)
                obj.x(obj.idxAngle) = SRSSUKF.SymmetricalAngle(obj.x(obj.idxAngle) + pivotAngle);
                obj.Xi(obj.idxAngle,:) = SRSSUKF.SymmetricalAngle(obj.Xi(obj.idxAngle,:) + repmat(pivotAngle, 1, obj.numSP));
            end
        end
    end
    properties(Access = private)
        % State/Covariance/Sigma-points
        x             % Augmented state vector.
        S             % Square-root covariance matrix.
        Xi            % List of sigma-points.
        dX            % Xi - x.
        % Dimensions
        nx            % Dimension of state vector (x).
        nxS           % Dimension of state vector (x) with respect to unvertainty S.
        L             % Dimension of augmented state vector.
        LS            % Dimension of augmented state vector with respect to uncertainty S.
        numSP         % Number of sigma-points.
        nw            % Dimension of process noise (w).
        % Weights
        w0            % Weighting for 0-th sigma-point.
        wi            % Weighting for i-th sigma-point, i > 0.
        srw0          % Square-root of w0.
        srwi          % Square-root of wi.
        Z             % Weighting matrix.
        % Process
        idxAngle      % Indices for angles.
        idxQuaternion % Indices for quaternions (q0).
        % Observations
        numObsv       % Number of observations.
        infoObsv      % 3-by-(numObsv) info matrix, column: yDim,vIndexOffset,vDim.
        % Misc.
        spU2D         % True if sigma-points are up-to-date, false otherwise.
    end
end


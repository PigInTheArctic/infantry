%% lqr_calc.m
% Discrete-time (sampled-data) LQR design for the gimbal yaw axis.
%
% Firmware path:
%   user/algorithm/src/LQR.cpp    - Lqr::Calc()
%   user/application/inc/gimbal.h - Gimbal::gimbal_lqr_
%
% Firmware control law (per axis, LQR.cpp currently reuses one gain pair for
% the yaw row and the pitch row):
%   tau      = -K(1) * e_theta - K(2) * e_omega
%   e_theta  = theta_ref - theta_meas
%   e_omega  = omega_ref - omega_meas
%
% This script designs K in the DISCRETE domain:
%   1. Build the continuous yaw plant J * theta_ddot + B * theta_dot = tau.
%   2. Sample it at Ts with zero-order hold -> x_{k+1} = Ad * x_k + Bd * u_k.
%   3. Solve the discrete-time Riccati equation with dlqr(), i.e. minimize
%        J = sum_k (x_k' * Q * x_k + u_k' * R * u_k),  u_k = -K * x_k
%   4. Verify the result with the DLQR closed-loop poles and a saturated step
%      simulation running on the SAME discrete model used for the design.
%
% Why discrete: the firmware evaluates the control law once per Ts, so the ZOH
% model plus the discrete Riccati solution matches the implemented loop, and
% the printed K can be pasted into LQR.cpp without a continuous/discrete
% mismatch.
%
% Workflow:
%   1. Edit J / Bv / Ts and the Q / R weights below.
%   2. Run.
%   3. Copy the printed row 0 (yaw) into user/algorithm/src/LQR.cpp; re-run
%      with the pitch J / Bv if the pitch row should get its own gain pair.
%
% Feedforward torque, ESO, integral action, bias and output limits stay
% firmware-side and are not part of the Riccati solve.

clear; clc; close all;

%% Plant model and firmware limits
%J  = 0.0201187059;   % alternative identification, kg*m^2
%Bv = 0.207791924;    % alternative identification, N*m*s/rad
J  = 0.0342594124;   % kg*m^2
Bv = 0.242238596;    % N*m*s/rad
Ts = 0.001;          % sampling period, s
torqueLimit = 1.6;   % DM motor torque limit, N*m
torqueSlewRate = 7000.0;  % firmware torque slew rate, N*m/s

% Gains currently used in user/algorithm/src/LQR.cpp (row 0 yaw, row 1 pitch).
currentKTheta = 20.0;
currentKOmega = 2.7156752;

%% Tunable QR weights (discrete-time cost)
% State is x = [theta_error_rad; omega_error_rad_s].
% Increase Q(1,1) for tighter angle tracking, Q(2,2) for more damping,
% increase R for a softer torque demand.
Q = [400.0, 0.0;
       0.0, 1.0];
R = 1.0;

%% Manual gain override
% Keep false when you want the QR weights to drive the printed K.
useManualTargetGain = false;
targetKTheta = currentKTheta;
targetKOmega = currentKOmega;

%% Offline sanity-step settings
stepRefDeg = 5.0;
simTimeS = 1.0;
settleBandRatio = 0.02;
enablePlot = false;

%% Continuous plant: J * theta_ddot + B * theta_dot = tau
A = [0, 1;
     0, -Bv / J];
Bu = [0;
      1 / J];

validateQrMatrices(Q, R);
Q = 0.5 * (Q + Q.');

%% Discretize with zero-order hold
if exist('ss', 'file') ~= 2 || exist('c2d', 'file') ~= 2 || ...
   exist('dlqr', 'file') ~= 2 || exist('lqr', 'file') ~= 2
    error(['Control System Toolbox functions ss(), c2d(), dlqr() and lqr() ', ...
           'are required for this discrete-time tuning script.']);
end

sysC = ss(A, Bu, eye(2), zeros(2, 1));
sysD = c2d(sysC, Ts, 'zoh');
Ad = sysD.A;
Bd = sysD.B;

if rank([Bd, Ad * Bd]) < size(Ad, 1)
    error('Discrete yaw plant is not controllable. Check J, Bv and Ts.');
end

%% Discrete-time LQR
% dlqr() solves the discrete algebraic Riccati equation
%   Ad'*S*Ad - S - (Ad'*S*Bd)*(Bd'*S*Bd + R)^-1*(Bd'*S*Ad) + Q = 0
% and returns K = (Bd'*S*Bd + R)^-1 * Bd'*S*Ad.
Rmat = R * eye(size(Bd, 2));
if useManualTargetGain
    K = [targetKTheta, targetKOmega];
    S = NaN(2);
    gainSource = 'manual targetKTheta / targetKOmega';
else
    [K, S, ~] = dlqr(Ad, Bd, Q, Rmat);
    gainSource = 'discrete CARE (dlqr on the ZOH model)';
end
Ktheta = K(1);
Komega = K(2);

% Continuous design is kept for reference only: it optimizes a different cost
% and is NOT the gain the sampled loop implements.
[Kc, Pc, polesC] = lqr(A, Bu, Q, R);

%% Closed-loop pole check
discretePoles = eig(Ad - Bd * K);   % recomputed, independent of dlqr output
poleMagnitudes = abs(discretePoles);
isStable = all(poleMagnitudes < 1.0);
if ~isStable
    warning('Discrete closed-loop poles are outside the unit circle: check Q/R.');
end
continuousEquivalentPoles = log(discretePoles) / Ts;   % principal branch
poleInfo = estimatePoleMetrics(continuousEquivalentPoles);

%% Saturated step simulation on the discrete model without FF/ESO/bias
t = 0:Ts:simTimeS;
thetaRef = deg2rad(stepRefDeg);
x = zeros(2, numel(t));
u = zeros(1, numel(t));
uRaw = zeros(1, numel(t));
torqueClampActive = false(1, numel(t));
slewActive = false(1, numel(t));
uLast = 0.0;

for k = 1:numel(t)-1
    e = x(:, k) - [thetaRef; 0];
    uRaw(k) = -K * e;

    uLimited = clampScalar(uRaw(k), -torqueLimit, torqueLimit);
    torqueClampActive(k) = (abs(uLimited - uRaw(k)) > 1e-12);

    maxDeltaU = torqueSlewRate * Ts;
    uSlew = clampScalar(uLimited, uLast - maxDeltaU, uLast + maxDeltaU);
    slewActive(k) = (abs(uSlew - uLimited) > 1e-12);

    u(k) = uSlew;
    uLast = u(k);
    x(:, k+1) = Ad * x(:, k) + Bd * u(k);
end
u(end) = u(end-1);
uRaw(end) = uRaw(end-1);
torqueClampActive(end) = torqueClampActive(end-1);
slewActive(end) = slewActive(end-1);

thetaDeg = rad2deg(x(1, :));
omegaRadS = x(2, :);
thetaErrDeg = thetaDeg - stepRefDeg;
settleBandDeg = settleBandRatio * max(abs(stepRefDeg), eps);
settleTime = findSettlingTime(t, thetaErrDeg, settleBandDeg);
stepDir = sign(stepRefDeg);
if stepDir == 0
    stepDir = 1;
end
overshootDeg = max(0, max(stepDir * thetaDeg) - abs(stepRefDeg));
finalErrorDeg = thetaErrDeg(end);
maxTorque = max(abs(u));

%% Print results
fprintf('================ Gimbal discrete-time QR-LQR tuning ================\n');
fprintf('Firmware model: J*theta_ddot + B*theta_dot = tau\n');
fprintf('J               = %.9g kg*m^2\n', J);
fprintf('Bv              = %.9g N*m*s/rad\n', Bv);
fprintf('Ts              = %.9g s\n', Ts);
fprintf('torqueLimit     = %.9g N*m\n', torqueLimit);
fprintf('torqueSlewRate  = %.9g N*m/s\n', torqueSlewRate);

fprintf('\nDiscrete-time cost weights used for this run:\n');
fprintf('Q =\n'); disp(Q);
fprintf('R =\n'); disp(R);

fprintf('\nContinuous plant A/Bu:\n');
fprintf('A =\n'); disp(A);
fprintf('Bu =\n'); disp(Bu);
fprintf('Discrete plant Ad (ZOH, Ts = %.9g s):\n', Ts);
disp(Ad);
fprintf('Discrete plant Bd:\n');
disp(Bd);

fprintf('\nK source = %s\n', gainSource);
fprintf('Discrete LQR gain: K = [%.9g, %.9g]\n', Ktheta, Komega);
fprintf('tau_fb = -K * [e_theta; e_omega]\n');

fprintf('\nDiscrete closed-loop poles eig(Ad - Bd*K):\n');
disp(discretePoles.');
fprintf('pole magnitudes:\n');
disp(poleMagnitudes.');
fprintf('stable (all |z| < 1): %d\n', isStable);
fprintf('equivalent continuous poles log(z)/Ts:\n');
disp(continuousEquivalentPoles.');
fprintf('estimated wn   = %.9g rad/s\n', poleInfo.wn);
fprintf('estimated zeta = %.9g\n', poleInfo.zeta);
fprintf('Discrete Riccati solution S:\n');
disp(S);

fprintf('\nContinuous LQR reference (different cost domain, do not paste):\n');
fprintf('K_cont = [%.9g, %.9g]\n', Kc(1), Kc(2));
fprintf('continuous closed-loop poles:\n');
disp(polesC.');
fprintf('Continuous Riccati solution P:\n');
disp(Pc);

fprintf('\nCurrent firmware gains (user/algorithm/src/LQR.cpp):\n');
fprintf('current k[0] = [%.9g, %.9g]  (yaw row)\n', currentKTheta, currentKOmega);
fprintf('ratio vs new K: theta %.6f, omega %.6f\n', ...
        Ktheta / currentKTheta, Komega / currentKOmega);

fprintf('\nC++ copy block for LQR.cpp:\n');
fprintf('const float k[2][2] = {\n');
fprintf(' { %.9gf, %.9gf },\n', Ktheta, Komega);
fprintf(' { %.9gf, %.9gf },\n', Ktheta, Komega);
fprintf('};\n');
fprintf('(row 0 = yaw, row 1 = pitch; re-run with the pitch J/Bv for its own pair)\n');

fprintf('\nFirmware context not solved by the QR weights:\n');
fprintf('Feedforward uses its own J/Bv and caps; Coulomb FF, ESO, LQI, bias and\n');
fprintf('final clamps remain firmware-side.\n');

fprintf('\nSanity step: %.3f deg, %.3f s\n', stepRefDeg, simTimeS);
fprintf('max torque             = %.6f N*m (%.2f%% of limit)\n', ...
        maxTorque, 100.0 * maxTorque / torqueLimit);
fprintf('torque clamp samples   = %d / %d\n', ...
        nnz(torqueClampActive), numel(torqueClampActive));
fprintf('torque slew samples    = %d / %d\n', nnz(slewActive), numel(slewActive));
fprintf('overshoot              = %.6f deg\n', overshootDeg);
fprintf('final error            = %.6f deg\n', finalErrorDeg);
fprintf('settle band            = +/- %.6f deg\n', settleBandDeg);
fprintf('settle time            = %.6f s\n', settleTime);

%% Optional plots
if enablePlot
    figure('Name', 'Gimbal discrete LQR saturated step');

    subplot(3, 1, 1);
    plot(t, thetaDeg, 'LineWidth', 1.2); hold on;
    yline(stepRefDeg, '--', 'LineWidth', 1.0);
    yline(stepRefDeg + settleBandDeg, ':');
    yline(stepRefDeg - settleBandDeg, ':');
    grid on;
    ylabel('theta (deg)');
    title('Discrete LQR feedback-only step check');
    legend('theta', 'ref', '2% band', 'Location', 'best');

    subplot(3, 1, 2);
    plot(t, omegaRadS, 'LineWidth', 1.2);
    grid on;
    ylabel('omega (rad/s)');

    subplot(3, 1, 3);
    plot(t, u, 'LineWidth', 1.2); hold on;
    plot(t, uRaw, ':', 'LineWidth', 1.0);
    yline(torqueLimit, '--r');
    yline(-torqueLimit, '--r');
    grid on;
    ylabel('tau (N*m)');
    xlabel('time (s)');
    legend('limited tau', 'raw tau', 'limit', 'Location', 'best');
end

function y = clampScalar(x, lower, upper)
    y = min(max(x, lower), upper);
end

function validateQrMatrices(Q, R)
    if ~isequal(size(Q), [2, 2])
        error('Q must be a 2x2 matrix for x = [theta_error; omega_error].');
    end
    if ~isscalar(R)
        error('R must be a scalar / 1x1 matrix for yaw torque input.');
    end
    if any(~isfinite(Q(:))) || ~isfinite(R)
        error('Q and R must contain finite numeric values.');
    end
    if max(max(abs(Q - Q.'))) > 1e-9
        error('Q must be symmetric. Use Q = 0.5 * (Q + Q.'') if needed.');
    end
    if min(eig(Q)) < -1e-9
        error('Q must be positive semidefinite.');
    end
    if R <= 0.0
        error('R must be positive.');
    end
end

function settleTime = findSettlingTime(t, err, band)
    settleTime = NaN;
    inside = abs(err) <= band;
    for idx = 1:numel(t)
        if all(inside(idx:end))
            settleTime = t(idx);
            return;
        end
    end
end

function info = estimatePoleMetrics(poles)
    finitePoles = poles(isfinite(poles));
    info = struct('wn', NaN, 'zeta', NaN);
    if isempty(finitePoles)
        return;
    end

    complexIdx = find(abs(imag(finitePoles)) > 1e-9, 1, 'first');
    if ~isempty(complexIdx)
        p = finitePoles(complexIdx);
        sigma = -real(p);
        wd = abs(imag(p));
        info.wn = sqrt(sigma^2 + wd^2);
        info.zeta = sigma / max(info.wn, eps);
        return;
    end

    [~, idx] = max(real(finitePoles));
    dominantPole = finitePoles(idx);
    info.wn = max(-real(dominantPole), 0.0);
    info.zeta = 1.0;
end

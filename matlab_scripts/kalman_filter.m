%% kalman_filter.m
%
%  Offline verification of the 3-state vertical Kalman filter, using the
%  Control System Toolbox's kalman() + lsim() instead of a hand-written
%  for loop.  This version is set up for a STATIONARY sensor bench test:
%
%     The board must sit motionless for the entire capture.
%     A correctly-tuned filter should then report:
%        alt   ≈ 0 m      (± baro noise band)
%        vel   ≈ 0 m/s    (flat)
%        bias  → mean(a_z_world)   within a few seconds
%
%     Any sustained altitude drift means either the baro data is drifting
%     (hardware) or R / Q are mis-tuned for the current noise regime.
%
%  Toolbox requirement:  Control System Toolbox (for kalman() and ss()).
%
%  Line format expected on the COM port (CR/LF terminated):
%       "S: qw, qx, qy, qz, ax, ay, az, baro_alt"
%
%  ── Pipeline ──────────────────────────────────────────────────────────
%    1.  Capture N samples at 100 Hz
%    2.  Vectorised a_z_world  (quaternion rotation, no for-loop)
%    3.  Pre-process baro : median-despike + mean-zero the first window
%    4.  AUTO-TUNE Q and R from the captured stationary data so the
%        filter matches whatever the sensors are actually doing today
%    5.  Seed the filter state (bias ← observed accel mean)
%    6.  Build ss() plant; design steady-state KF with kalman()
%    7.  Run the whole record in one shot via lsim()
%    8.  Plot filtered vs. unfiltered altitude, velocity, accel
%
%  Author: Diego

clear; clc; close all;

%% 1. Configuration & Serial Capture ─────────────────────────────────────
PORT            = "COM9";
BAUD            = 115200;
CAPTURE_SECONDS = 20;          % 20 s minimum for bias to converge
DT              = 0.01;        % 100 Hz predict step
MAX_SAMPLES     = round(CAPTURE_SECONDS / DT) + 500;

AUTO_TUNE_RQ    = true;        % measure R and σ_az² from this capture
BARO_DESPIKE_W  = 5;           % median-filter window for baro (samples)
ZERO_WINDOW_S   = 2;           % seconds at start used for zero / bias seed

try
    s = serialport(PORT, BAUD);
    configureTerminator(s, "CR/LF");
    flush(s);
catch
    error('Could not open %s. Is another program using it?', PORT);
end

fprintf('Capturing %d s of data...  KEEP THE SENSOR ABSOLUTELY STATIONARY.\n', ...
        CAPTURE_SECONDS);

qw = zeros(MAX_SAMPLES,1); qx = zeros(MAX_SAMPLES,1);
qy = zeros(MAX_SAMPLES,1); qz = zeros(MAX_SAMPLES,1);
ax = zeros(MAX_SAMPLES,1); ay = zeros(MAX_SAMPLES,1); az = zeros(MAX_SAMPLES,1);
baro_alt = zeros(MAX_SAMPLES,1);

idx = 1; tic;
while toc < CAPTURE_SECONDS && idx <= MAX_SAMPLES
    str = readline(s);
    if startsWith(str, "S:")
        v = sscanf(str, 'S: %f,%f,%f,%f,%f,%f,%f,%f');
        if numel(v) == 8
            qw(idx)=v(1); qx(idx)=v(2); qy(idx)=v(3); qz(idx)=v(4);
            ax(idx)=v(5); ay(idx)=v(6); az(idx)=v(7);
            baro_alt(idx)=v(8);
            idx = idx + 1;
        end
    end
end
clear s;

N = idx - 1;
fprintf('Capture complete — %d samples (%.2f s).\n', N, N*DT);

% Trim & time vector
qw = qw(1:N); qx = qx(1:N); qy = qy(1:N); qz = qz(1:N);
ax = ax(1:N); ay = ay(1:N); az = az(1:N); baro_alt_raw = baro_alt(1:N);
t  = (0:N-1)' * DT;

%% 2. Vectorised world-Z acceleration ───────────────────────────────────
a_z_world = 2.*(qx.*qz - qw.*qy).*ax ...
          + 2.*(qy.*qz + qw.*qx).*ay ...
          + (1 - 2.*(qx.^2 + qy.^2)).*az;

%% 3. Baro pre-processing (despike + robust zero) ───────────────────────
% Median filter kills the large-amplitude spikes that a single-sample
% zero-reference would otherwise lock on to.
baro_despiked = medfilt1(baro_alt_raw, BARO_DESPIKE_W, 'truncate');

% Zero reference: mean of the first ZERO_WINDOW_S seconds (robust to the
% first-sample outlier and to initial settling).
n_zero = min(N, round(ZERO_WINDOW_S / DT));
baro_zero_ref = mean(baro_despiked(1:n_zero));
baro_alt      = baro_despiked - baro_zero_ref;

%% 4. Auto-tune Q and R from the captured stationary record ─────────────
% Any process we measure RIGHT NOW from a stationary board represents the
% sensor's actual noise regime — which may differ run-to-run (temperature,
% power-supply, vibration floor).  Auto-tuning lets the filter match what
% the sensor is doing today rather than what it did last week.
%
% σ_az² : variance of a_z_world   (true signal is 0 → variance = noise)
% q_bias: keep the literature-derived value; bias random walk is a long
%         timescale phenomenon that we can't estimate from 20 s.
% R     : variance of the zeroed, despiked baro (true signal is 0)

if AUTO_TUNE_RQ
    sigma_az_sq = var(a_z_world - mean(a_z_world));
    R           = var(baro_alt);
    q_bias      = 6.8263e-09;   % from accel_bias_allan.m — leave as-is
    fprintf('\n[AUTO-TUNE] measured from this capture:\n');
    fprintf('    σ_az²  = %.4e  (m/s²)²    (was hard-coded 3.8770e-04)\n', sigma_az_sq);
    fprintf('    R      = %.4e  m²         (baro noise variance)\n',  R);
    fprintf('    √R     = %.4f  m          (1σ baro altitude noise)\n',  sqrt(R));
else
    sigma_az_sq = 3.8770e-04;
    q_bias      = 6.8263e-09;
    R           = 0.1673287;
end

% Discrete process-noise matrix Q (same form as the C code)
Q_accel = sigma_az_sq * [ (DT^4)/4,  (DT^3)/2, 0 ;
                          (DT^3)/2,   DT^2,    0 ;
                            0,         0,      0 ];
Q_bias_mat = diag([0, 0, q_bias]);
Q = Q_accel + Q_bias_mat;

fprintf('\n--- Q paste block for main.c ---\n');
fprintf('static const float32_t KZ_Q[9] = {\n');
fprintf('    %.4ef,   %.4ef,   0.0f,\n', Q(1,1), Q(1,2));
fprintf('    %.4ef,   %.4ef,   0.0f,\n', Q(2,1), Q(2,2));
fprintf('    0.0f,          0.0f,          %.4ef\n', Q(3,3));
fprintf('};\n');

%% 5. Seed filter state from the stationary initial window ──────────────
% The first ZERO_WINDOW_S seconds are a "calibration" window.  Since the
% board is known stationary, we can directly estimate the bias there
% (it's the mean of a_z_world).  Altitude and velocity start at 0.
bias_seed = mean(a_z_world(1:n_zero));
x0 = [0; 0; bias_seed];

fprintf('\n[SEED] initial state from first %.1f s of data:\n', ZERO_WINDOW_S);
fprintf('    h0 = %.4f m   v0 = %.4f m/s   b0 = %.4f m/s²\n', ...
        x0(1), x0(2), x0(3));

%% 6. Build discrete LTI plant for kalman() ─────────────────────────────
F  = [1, DT, -0.5*DT^2 ;
      0,  1, -DT       ;
      0,  0,  1        ];
Bu = [0.5*DT^2 ; DT ; 0];
Bw = eye(3);
H  = [1, 0, 0];

sys = ss(F, [Bu, Bw], H, zeros(1, 1 + 3), DT);
sys.InputName  = {'u_az','w1','w2','w3'};
sys.OutputName = {'y_baro'};
sys.StateName  = {'h','v','b'};

%% 7. Design steady-state Kalman estimator ──────────────────────────────
[kest, L_gain, P_ss] = kalman(sys, Q, R, [], 1, 1);

fprintf('\n--- Steady-state Kalman gain  L ---\n');   disp(L_gain);
fprintf('--- Steady-state covariance   P_∞ ---\n');   disp(P_ss);

%% 8. Run the whole record with lsim (no for-loop) ──────────────────────
U    = [a_z_world, baro_alt];         % N × 2
Yall = lsim(kest, U, t, x0);          % N × 4, seeded initial state

y_est   = Yall(:,1);
alt_kf  = Yall(:,2);
vel_kf  = Yall(:,3);
bias_kf = Yall(:,4);

%% 9. Unfiltered reference signals (for comparison on the plots) ────────
%   Same as before: raw baro, pure-accel double-integration, baro-diff,
%   accel-integration.  We also subtract the seeded bias from the
%   accel-integrate trace so it shares the KF's frame.
a_unb           = a_z_world - bias_seed;              % bias-removed accel
alt_accel_only  = cumtrapz(t, cumtrapz(t, a_unb));    % ∫∫ a dt dt
vel_accel_only  = cumtrapz(t, a_unb);                 % ∫ a dt
vel_baro_diff   = [0; diff(baro_alt)] / DT;

%% 10. Plots ─────────────────────────────────────────────────────────────
% One figure per variable (altitude / velocity / bias). Each figure uses
% subplots to separate the filtered KF trace from the unfiltered reference
% signals, so the filter's effect is easy to see without the traces piling
% on top of each other.

% ── Figure 1 : Altitude ─────────────────────────────────────────────────
figure('Name','Altitude — KF vs unfiltered references', ...
       'Position', [100 100 1000 820]);

subplot(3,1,1);
plot(t, alt_kf, 'b', 'LineWidth',1.6);
yline(0, 'k:');  grid on;
ylabel('Altitude  [m]');
title(sprintf(['Filtered KF altitude  ', ...
               '(DT=%.0f ms, √R=%.3f m, σ_{az}=%.4f m/s²)'], ...
               DT*1e3, sqrt(R), sqrt(sigma_az_sq)));

subplot(3,1,2);
plot(t, baro_alt_raw - baro_zero_ref, 'Color',[0.80 0.80 0.80], 'LineWidth',0.5, ...
     'DisplayName','raw baro (zeroed)');  hold on;
plot(t, baro_alt, 'Color',[0.35 0.35 0.35], 'LineWidth',0.8, ...
     'DisplayName','baro despiked');
yline(0, 'k:', 'HandleVisibility','off');  grid on;
ylabel('Altitude  [m]');  legend('Location','best');
title('Unfiltered: barometric altitude');

subplot(3,1,3);
plot(t, alt_accel_only, 'Color',[1.00 0.55 0.10], 'LineWidth',0.8);
yline(0, 'k:');  grid on;
ylabel('Altitude  [m]');  xlabel('Time  [s]');
title('Unfiltered: \int\int (a_{z,world} - b_{seed}) dt dt');

% ── Figure 2 : Velocity ─────────────────────────────────────────────────
figure('Name','Velocity — KF vs unfiltered references', ...
       'Position', [130 130 1000 820]);

subplot(3,1,1);
plot(t, vel_kf, 'r', 'LineWidth',1.6);
yline(0, 'k:');  grid on;
ylabel('Velocity  [m/s]');
title('Filtered KF velocity — should stay near zero on a stationary board');

subplot(3,1,2);
plot(t, vel_baro_diff, 'Color',[0.55 0.55 0.55], 'LineWidth',0.5);
yline(0, 'k:');  grid on;
ylabel('Velocity  [m/s]');
title('Unfiltered: d(baro)/dt');

subplot(3,1,3);
plot(t, vel_accel_only, 'Color',[1.00 0.55 0.10], 'LineWidth',0.8);
yline(0, 'k:');  grid on;
ylabel('Velocity  [m/s]');  xlabel('Time  [s]');
title('Unfiltered: \int (a_{z,world} - b_{seed}) dt');

% ── Figure 3 : Accel bias ───────────────────────────────────────────────
figure('Name','Accel bias — KF estimate vs raw a_{z,world}', ...
       'Position', [160 160 1000 820]);

subplot(3,1,1);
plot(t, bias_kf, 'g', 'LineWidth',1.6);  hold on;
yline(bias_seed, ':k', sprintf('bias seed = %+.4f', bias_seed), ...
      'LabelHorizontalAlignment','left');
grid on;
ylabel('Bias  [m/s^2]');
title('Filtered KF accel bias');

subplot(3,1,2);
plot(t, a_z_world, 'Color',[0.55 0.55 0.55], 'LineWidth',0.6);  hold on;
yline(mean(a_z_world), ':k', sprintf('mean = %+.4f', mean(a_z_world)), ...
      'LabelHorizontalAlignment','left');
grid on;
ylabel('a_{z,world}  [m/s^2]');
title('Unfiltered: world-Z acceleration');

subplot(3,1,3);
plot(t, a_z_world - bias_kf, 'Color',[0.20 0.50 0.20], 'LineWidth',0.6);
yline(0, 'k:');  grid on;
ylabel('a_{z,world} − b_{KF}  [m/s^2]');  xlabel('Time  [s]');
title('Residual acceleration after KF bias removal');

%% 11. Stationary-board diagnostics ──────────────────────────────────────
% Evaluate the filter only AFTER the transient (first 2 s), so the initial
% gain does not dominate the summary numbers.
mask = t >= ZERO_WINDOW_S;

alt_rms     = rms(alt_kf(mask));
alt_peak    = max(abs(alt_kf(mask)));
alt_drift   = alt_kf(end) - alt_kf(find(mask,1,'first'));
vel_rms     = rms(vel_kf(mask));
vel_peak    = max(abs(vel_kf(mask)));
innov_sigma = std(baro_alt - alt_kf);

% Slope of a linear fit to the despiked baro — if non-trivial, the baro
% itself is drifting (hardware/firmware), not the filter.
p_baro = polyfit(t, baro_alt, 1);

fprintf('\n--- Stationary-board diagnostics (t ≥ %.1f s) ---\n', ZERO_WINDOW_S);
fprintf('  KF altitude  RMS  :  %8.4f  m\n',   alt_rms);
fprintf('  KF altitude  peak :  %8.4f  m\n',   alt_peak);
fprintf('  KF altitude  drift (end − start of window) : %+8.4f m  over %.1f s\n', ...
        alt_drift, t(end) - ZERO_WINDOW_S);
fprintf('  KF velocity  RMS  :  %8.4f  m/s\n', vel_rms);
fprintf('  KF velocity  peak :  %8.4f  m/s\n', vel_peak);
fprintf('  baro innov σ (vs KF)  :  %8.4f m\n', innov_sigma);
fprintf('  baro linear drift rate:  %+.4f m/s  (slope of despiked baro)\n', p_baro(1));

if abs(p_baro(1)) > 0.02
    fprintf(['\n  ⚠  Baro has a linear trend of %+.3f m/s — ', ...
             'this is a HARDWARE/FIRMWARE issue, not a filter one.\n', ...
             '     Check: reference pressure settling, sensor warm-up, ', ...
             'HVAC air currents, or the MCU''s p_sea_pa drift.\n'], ...
             p_baro(1));
end

fprintf('\n  final altitude : %+8.4f m   (should be ≈ 0)\n', alt_kf(end));
fprintf('  final velocity : %+8.4f m/s (should be ≈ 0)\n', vel_kf(end));
fprintf('  final bias     : %+8.4f m/s² (should ≈ mean(a_z) = %+.4f)\n', ...
        bias_kf(end), mean(a_z_world));

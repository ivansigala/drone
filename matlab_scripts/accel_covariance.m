%% accel_covariance.m
%  Accelerometer noise characterisation — reads directly from a COM port.
%
%  Expected line format (one sample per line, CR/LF terminated):
%       : <ax>, <ay>, <az>
%  i.e. the output of:
%       PRINTF(": %.2f, %.2f, %.2f\r\n", ax, ay, az);
%  Any leading prefix before the ':' is tolerated.  Lines that don't
%  match are silently skipped, so extra debug prints won't break it.
%
%  HOW TO USE
%  ----------
%   1. Keep the drone absolutely still on the bench (no fans, no vibration).
%   2. Close any terminal that holds COM9 (PuTTY, MCUXpresso, Tera Term).
%   3. Run this script. It captures for CAPTURE_SECONDS and then stops.
%   4. Paste the printed q_alt / q_vel into kalman_z_init().
%
%  Requires MATLAB R2019b or newer (uses serialport, not the legacy serial).
%
%  Author : Diego
% -------------------------------------------------------------------------
clear; clc; close all;

%% ----- USER SETTINGS ----------------------------------------------------
PORT             = "COM9";
BAUD             = 115200;
CAPTURE_SECONDS  = 60;           % how long to log
SAMPLE_DT        = 0.01;         % predict step [s] — must match kalman_z_init
DROP_FIRST       = 200;          % drop N initial samples (IMU warm-up)
MAX_SAMPLES      = 200000;       % preallocation cap
TERMINATOR       = "CR/LF";      % match your PRINTF ("\r\n" → CR/LF, "\n" → LF)
% -------------------------------------------------------------------------

%% ----- OPEN PORT AND CAPTURE -------------------------------------------
s = serialport(PORT, BAUD);
configureTerminator(s, TERMINATOR);
s.Timeout = 0.2;          % short timeout so the outer loop can exit cleanly
flush(s);

pat    = ':\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)';
buf    = nan(MAX_SAMPLES, 3);
k      = 0;
tStart = tic;

fprintf('Capturing %g s from %s @ %d baud ... (Ctrl+C to abort)\n', ...
        CAPTURE_SECONDS, PORT, BAUD);

while toc(tStart) < CAPTURE_SECONDS && k < MAX_SAMPLES
    try
        line = readline(s);
    catch
        continue;           % read timeout, keep going
    end
    tk = regexp(line, pat, 'tokens', 'once');
    if ~isempty(tk)
        k = k + 1;
        buf(k,:) = str2double(tk);
    end
end
clear s;                    % releases the COM port

if k == 0
    error('No samples matched — check PRINTF format and TERMINATOR.');
end

A = buf(1:k, :);
if DROP_FIRST > 0 && DROP_FIRST < size(A,1)
    A = A(DROP_FIRST+1:end, :);
end
N = size(A,1);
fprintf('Captured %d parsed samples (%.2f s @ %.1f Hz effective).\n\n', ...
        N, toc(tStart), N/toc(tStart));

%% ----- STATISTICS -------------------------------------------------------
mu    = mean(A, 1);
var_  = var (A, 0, 1);
sigma = sqrt(var_);
C     = cov(A);

axisN = 'xyz';
fprintf('Per-axis statistics\n');
fprintf('  axis    mean [m/s^2]     sigma [m/s^2]       sigma^2 [(m/s^2)^2]\n');
fprintf('  --------------------------------------------------------------\n');
for i = 1:3
    fprintf('   %c    %+10.6f      %10.6f        %12.6e\n', ...
            axisN(i), mu(i), sigma(i), var_(i));
end

fprintf('\n3x3 sample covariance matrix  [(m/s^2)^2]:\n');
disp(C);

R_corr = C ./ (sigma.' * sigma);
fprintf('Correlation matrix (should be ~I for a clean sensor):\n');
disp(R_corr);

%% ----- KALMAN-Z TUNING SUGGESTION ---------------------------------------
sigma_az_sq = var_(3);
dt          = SAMPLE_DT;

% Accel-driven white-noise Q sub-block (rows/cols 0..1: altitude, velocity)
Q_accel = sigma_az_sq * [ dt^4/4  dt^3/2  0 ;
                          dt^3/2  dt^2    0 ;
                          0       0       0 ];

% Placeholder for the bias random-walk variance — extracted separately by
% accel_bias_allan.m over a much longer static capture.
q_bias_placeholder = 1e-8;

Q_full = Q_accel;
Q_full(3,3) = q_bias_placeholder;

fprintf('\n=== Suggested Kalman-Z process-noise tuning (dt = %.3f s) ===\n', dt);
fprintf('  sigma_az^2 = %.6e  (m/s^2)^2\n\n', sigma_az_sq);
fprintf('  Q_accel (3x3, accel white noise only — fill in q_bias separately):\n');
disp(Q_accel);

fprintf('  Copy-paste block for main.c (Q_accel + placeholder q_bias):\n\n');
fprintf('static const float32_t KZ_Q[9] = {\n');
fprintf('    /* row 0:  h / {h, v, b}  */\n');
fprintf('    %.4ef,   %.4ef,   0.0f,\n',       Q_full(1,1), Q_full(1,2));
fprintf('    /* row 1:  v / {h, v, b}  */\n');
fprintf('    %.4ef,   %.4ef,   0.0f,\n',       Q_full(2,1), Q_full(2,2));
fprintf('    /* row 2:  b / {h, v, b}   ← replace %.1e with q_bias from Allan script */\n', ...
        q_bias_placeholder);
fprintf('    0.0f,          0.0f,          %.4ef\n',  Q_full(3,3));
fprintf('};\n');

%% ----- PLOTS ------------------------------------------------------------
t = (0:N-1) * SAMPLE_DT;

figure('Name','Accel static time series','Color','w');
for i = 1:3
    subplot(3,1,i);
    plot(t, A(:,i), 'LineWidth', 0.5); hold on;
    yline(mu(i),             'r--', sprintf('\\mu = %+.4f', mu(i)));
    yline(mu(i) + sigma(i), ':k');
    yline(mu(i) - sigma(i), ':k');
    ylabel(sprintf('a_%c  [m/s^2]', axisN(i)));
    grid on;
    if i == 1, title('Accelerometer static output'); end
end
xlabel('Time [s]');

figure('Name','Accel histograms','Color','w');
for i = 1:3
    subplot(1,3,i);
    histogram(A(:,i), 50, 'Normalization', 'pdf'); hold on;
    xg = linspace(min(A(:,i)), max(A(:,i)), 200);
    plot(xg, normpdf(xg, mu(i), sigma(i)), 'r', 'LineWidth', 1.4);
    title(sprintf('a_%c   \\sigma = %.4f', axisN(i), sigma(i)));
    xlabel('m/s^2'); ylabel('pdf'); grid on;
end

figure('Name','Running variance (stationarity check)','Color','w');
win = max(100, round(0.05*N));
for i = 1:3
    subplot(3,1,i);
    plot(t, movvar(A(:,i), win), 'LineWidth', 0.7);
    ylabel(sprintf('var(a_%c)', axisN(i)));
    grid on;
    if i == 1
        title(sprintf('Running variance (window = %d samples ≈ %.2f s)', ...
              win, win*SAMPLE_DT));
    end
end
xlabel('Time [s]');

fprintf('\nDone.\n');

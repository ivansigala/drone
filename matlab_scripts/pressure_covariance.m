%% pressure_covariance.m
%  BMP581 pressure noise characterisation — reads directly from a COM port.
%
%  Expected line format (one sample per line, CR/LF terminated):
%       : <pressure_pa>, <temp_c>
%  i.e. the output of the BMP_VALIDATION_MODE polling loop in main.c:
%       PRINTF(": %.3f, %.2f\r\n", pressure_pa, temperature_c);
%
%  If you only print pressure (": %.2f\r\n"), set HAS_TEMP = false and the
%  script will fall back to USE_TEMP_C for the altitude conversion.
%
%  WHAT THIS SCRIPT DOES
%  ---------------------
%  A raw pressure variance in Pa^2 is not what the Kalman filter wants —
%  r_baro in kalman_z_init() is in m^2 (measurement noise on *altitude*,
%  not pressure).  This script:
%    1. Captures pressure (and optionally temperature) over serial.
%    2. Sanity-filters samples to the BMP581 operating range
%       (~30–125 kPa, T in [-40,+85] degC) so transient garbage from
%       the IIR settle / SPI desync cannot poison the variance.
%    3. Converts every surviving sample to altitude using the same
%       hypsometric formula as kalman_z.c
%       (h = (R / (M·g)) · T_K · ln(P_sea / P)).
%    4. Computes sigma^2_altitude directly — that is your r_baro.
%    5. Also reports sigma^2_pressure for reference.
%
%  HOW TO USE
%  ----------
%   1. In main.c, set  #define BMP_VALIDATION_MODE 1  and flash.
%   2. Put the BMP581 on the bench, temperature-stable, away from drafts.
%      The firmware already warms up ~3 s internally so the strong IIR
%      (coef=127, τ ≈ 2.5 s @ 50 Hz) is converged before the first line
%      hits the serial port.
%   3. Close any other terminal holding the COM port.
%   4. Run this script.  Paste r_baro into kalman_z_init().
%   5. Set BMP_VALIDATION_MODE back to 0 for normal flight.
%
%  Requires MATLAB R2019b or newer.
%
%  Author : Diego
% -------------------------------------------------------------------------
clear; clc; close all;

%% ----- USER SETTINGS ----------------------------------------------------
PORT             = "COM9";
BAUD             = 115200;
CAPTURE_SECONDS  = 120;          % baro noise is slow — capture longer
HAS_TEMP         = true;         % true if line is "Pa, degC"; false if Pa only
USE_TEMP_C       = 25.0;         % fallback temp if HAS_TEMP = false  [degC]
DROP_FIRST       = 25;           % small extra trim — firmware already
                                 % does ~3 s of pre-warm flushing
MAX_SAMPLES      = 200000;
TERMINATOR       = "CR/LF";

% BMP581 datasheet operating range — anything outside this is silicon-
% impossible and almost certainly a transient/parse glitch we want to
% drop before computing variance.
P_MIN_PA         = 30000.0;      % 300 hPa  (≈ 9 km altitude)
P_MAX_PA         = 125000.0;     % 1250 hPa (≈ -700 m below sea level)
T_MIN_C          = -40.0;
T_MAX_C          =  85.0;
% -------------------------------------------------------------------------

%% ----- REGEX ------------------------------------------------------------
if HAS_TEMP
    pat = ':\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)';   % pressure, temp
    NCOL = 2;
else
    pat = ':\s*(-?\d+\.?\d*)';                        % pressure only
    NCOL = 1;
end

%% ----- OPEN PORT AND CAPTURE -------------------------------------------
s = serialport(PORT, BAUD);
configureTerminator(s, TERMINATOR);
s.Timeout = 0.5;
flush(s);

buf    = nan(MAX_SAMPLES, NCOL);
k      = 0;
tStart = tic;

fprintf('Capturing %g s from %s @ %d baud ... (Ctrl+C to abort)\n', ...
        CAPTURE_SECONDS, PORT, BAUD);

while toc(tStart) < CAPTURE_SECONDS && k < MAX_SAMPLES
    try
        line = readline(s);
    catch
        continue;
    end
    tk = regexp(line, pat, 'tokens', 'once');
    if ~isempty(tk)
        k = k + 1;
        buf(k,:) = str2double(tk);
    end
end
clear s;

if k == 0
    error('No samples matched — check PRINTF format, HAS_TEMP and TERMINATOR.');
end

D = buf(1:k, :);
if DROP_FIRST > 0 && DROP_FIRST < size(D,1)
    D = D(DROP_FIRST+1:end, :);
end
N_raw = size(D,1);
dt    = toc(tStart) / k;                  % effective sample period
fprintf('Captured %d raw parsed samples (%.2f s, Ts ≈ %.4f s → %.1f Hz).\n', ...
        N_raw, toc(tStart), dt, 1/dt);

% --- Sanity-range filter: drop physically impossible samples ----------
P_raw = D(:,1);
if HAS_TEMP
    T_raw = D(:,2);
else
    T_raw = USE_TEMP_C * ones(N_raw,1);
end

valid = (P_raw >= P_MIN_PA) & (P_raw <= P_MAX_PA) & ...
        (T_raw >= T_MIN_C ) & (T_raw <= T_MAX_C );
n_drop = sum(~valid);
if n_drop > 0
    fprintf('  dropped %d out-of-range samples (%.2f%%).\n', ...
            n_drop, 100*n_drop/N_raw);
end
if n_drop > N_raw/2
    warning(['More than half the samples were out of the BMP581 range. ' ...
             'The parse function or sensor configuration may still be wrong.']);
end

P_pa = P_raw(valid);
T_C  = T_raw(valid);
T_K  = T_C + 273.15;
N    = numel(P_pa);
fprintf('  using %d in-range samples for statistics.\n\n', N);

%% ----- PRESSURE STATISTICS ---------------------------------------------
mu_P    = mean(P_pa);
sigma_P = std (P_pa);
var_P   = var (P_pa);

fprintf('Pressure statistics:\n');
fprintf('   mean    = %10.3f Pa   (%.3f hPa)\n', mu_P, mu_P/100);
fprintf('   sigma   = %10.6f Pa\n',   sigma_P);
fprintf('   sigma^2 = %.6e Pa^2\n\n', var_P);

%% ----- CONVERT TO ALTITUDE (same pipeline as kalman_z.c) ---------------
% Hypsometric formula:  h = (R / (M·g)) · T_K · ln(P_sea / P)
R_DIV_MG = 29.2715;           % R / (M_air · g) = 8.314 / (0.028964 · 9.80665)
P_SEA    = mu_P;              % reference = mean pressure during capture
                              % (makes mean altitude ≈ 0, isolates noise)

alt_m = R_DIV_MG .* T_K .* log(P_SEA ./ P_pa);

mu_h    = mean(alt_m);
sigma_h = std (alt_m);
var_h   = var (alt_m);

fprintf('Altitude statistics (through the hypsometric formula):\n');
fprintf('   mean    = %+10.6f m    (should be ~0 by construction)\n', mu_h);
fprintf('   sigma   = %10.6f m\n',   sigma_h);
fprintf('   sigma^2 = %.6e m^2\n\n', var_h);

%% ----- KALMAN-Z TUNING SUGGESTION --------------------------------------
r_baro = var_h;

fprintf('=== Suggested Kalman-Z measurement noise ===\n');
fprintf('   r_baro = %.6e   [m^2]    ← plug into kalman_z_init()\n\n', r_baro);

% Sanity cross-check using the analytic sensitivity  dh/dP = -R·T / (M·g·P)
dhdP   = -R_DIV_MG * mean(T_K) / mu_P;       % m per Pa
r_anal = dhdP^2 * var_P;
fprintf('Sanity check via dh/dP sensitivity: r_baro ≈ %.6e m^2\n', r_anal);
fprintf('  (should match the direct value within a few %%).\n\n');

%% ----- PLOTS -----------------------------------------------------------
t = (0:N-1) * dt;

figure('Name','Pressure & altitude time series','Color','w');
subplot(2,1,1);
plot(t, P_pa, 'LineWidth', 0.6); hold on;
yline(mu_P, 'r--', sprintf('\\mu = %.2f Pa', mu_P));
ylabel('Pressure [Pa]'); grid on;
title('BMP581 static output');

subplot(2,1,2);
plot(t, alt_m, 'LineWidth', 0.6); hold on;
yline(0, 'r--');
yline(sigma_h,  ':k');
yline(-sigma_h, ':k');
ylabel('Altitude noise [m]'); xlabel('Time [s]'); grid on;
title(sprintf('Altitude through hypsometric formula  (\\sigma = %.4f m)', sigma_h));

figure('Name','Histograms','Color','w');
subplot(1,2,1);
histogram(P_pa, 50, 'Normalization', 'pdf'); hold on;
xg = linspace(min(P_pa), max(P_pa), 200);
plot(xg, normpdf(xg, mu_P, sigma_P), 'r', 'LineWidth', 1.4);
title(sprintf('Pressure   \\sigma = %.4f Pa', sigma_P));
xlabel('Pa'); ylabel('pdf'); grid on;

subplot(1,2,2);
histogram(alt_m, 50, 'Normalization', 'pdf'); hold on;
xg = linspace(min(alt_m), max(alt_m), 200);
plot(xg, normpdf(xg, 0, sigma_h), 'r', 'LineWidth', 1.4);
title(sprintf('Altitude   \\sigma = %.4f m', sigma_h));
xlabel('m'); ylabel('pdf'); grid on;

figure('Name','Running variance (stationarity check)','Color','w');
win = max(100, round(0.05*N));
subplot(2,1,1);
plot(t, movvar(P_pa, win), 'LineWidth', 0.8);
ylabel('var(P)  [Pa^2]'); grid on;
title(sprintf('Running variance (window = %d samples ≈ %.2f s)', ...
      win, win*dt));

subplot(2,1,2);
plot(t, movvar(alt_m, win), 'LineWidth', 0.8);
ylabel('var(h)  [m^2]'); xlabel('Time [s]'); grid on;

fprintf('Done.\n');

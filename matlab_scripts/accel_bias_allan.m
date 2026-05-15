% accel_bias_allan.m
%
% Characterises the bias random walk of the BNO085 SH2_LINEAR_ACCELERATION
% Z axis using the Allan deviation.  Prints the `q_bias` entry that goes
% into the 3×3 KZ_Q matrix of the 3-state vertical Kalman filter.
%
% ── Test procedure ──────────────────────────────────────────────────────
%   1. Place the board FLAT AND STATIONARY (not handheld) on a solid
%      surface.  Any mechanical vibration or creep will poison the slow τ
%      part of the Allan plot.
%   2. Flash firmware that prints
%         ": ax, ay, az\r\n"
%      from the SH2_LINEAR_ACCELERATION callback at 100 Hz.
%   3. Run this script.  5+ minutes are needed to see the bias region;
%      10+ minutes is better.  DO NOT TOUCH THE BOARD during capture.
%
% ── Physics  ────────────────────────────────────────────────────────────
%
%   log-log Allan deviation vs τ has three canonical regions:
%
%       slope -1/2   ──  velocity random walk      (N  [m/s²·√s])
%       slope  0     ──  bias instability          (flicker floor)
%       slope +1/2   ──  rate  random walk         (K  [m/s²/√s])
%
%   The +1/2 region is what we tune q_bias against.  For a discrete
%   random-walk bias b[k+1] = b[k] + w[k], with w ~ 𝒩(0, q_bias):
%
%           q_bias = K² · dt           (per-step variance, m²/s⁴)
%
% ── Tuning guidance ─────────────────────────────────────────────────────
%   * If the Allan plot is flat or decreasing over the full τ range, the
%     bias random walk is below the noise floor of this test.  Start with
%     q_bias ≈ 1e-9 and let the filter tune itself.
%   * If the plot shows clear +1/2 growth, read K from the fit line at
%     τ = 3 s; the script prints this for you.

clear;  clc;

% ─── Config ────────────────────────────────────────────────────────────
COM_PORT        = "COM9";
BAUD            = 115200;
CAPTURE_SECONDS = 300;     % 5 min default — raise to 600+ for better fit
DROP_FIRST      = 200;     % throw away first N lines (startup noise)
DT_S            = 0.01;    % sample period 100 Hz

EXPECTED_SAMPLES = CAPTURE_SECONDS * round(1/DT_S);

% ─── Open serial ───────────────────────────────────────────────────────
fprintf("Opening %s @ %d baud...\n", COM_PORT, BAUD);
s = serialport(COM_PORT, BAUD);
configureTerminator(s, "CR/LF");
flush(s);

% ─── Collect samples ───────────────────────────────────────────────────
fprintf("Capturing for %d s (DO NOT MOVE the board)...\n", CAPTURE_SECONDS);

ax_buf = zeros(EXPECTED_SAMPLES, 1);
ay_buf = zeros(EXPECTED_SAMPLES, 1);
az_buf = zeros(EXPECTED_SAMPLES, 1);

n_kept = 0;
n_seen = 0;
t_start = tic;

while toc(t_start) < CAPTURE_SECONDS
    line = readline(s);
    if strlength(line) == 0, continue; end

    txt = char(line);
    k = strfind(txt, ':');
    if isempty(k), continue; end
    body = strtrim(txt(k(1)+1:end));

    v = sscanf(body, '%f, %f, %f');
    if numel(v) ~= 3, continue; end

    n_seen = n_seen + 1;
    if n_seen <= DROP_FIRST, continue; end

    n_kept = n_kept + 1;
    if n_kept > EXPECTED_SAMPLES
        % grow buffers if the MCU is slightly faster than 100 Hz
        ax_buf(end+1,1) = v(1); %#ok<*AGROW>
        ay_buf(end+1,1) = v(2);
        az_buf(end+1,1) = v(3);
    else
        ax_buf(n_kept) = v(1);
        ay_buf(n_kept) = v(2);
        az_buf(n_kept) = v(3);
    end
end

clear s;

ax = ax_buf(1:n_kept);
ay = ay_buf(1:n_kept);
az = az_buf(1:n_kept);

fprintf("Captured %d samples (%.1f s).\n", n_kept, n_kept*DT_S);

% ─── Allan deviation (non-overlapping) ─────────────────────────────────
% log-spaced τ values up to N/10 so we keep ≥10 chunks for variance stats.
m_max    = floor(n_kept / 10);
m_vec    = unique(round(logspace(0, log10(m_max), 40)));
m_vec    = m_vec(m_vec >= 1);

tau_vec  = m_vec * DT_S;

allan_ax = zeros(size(m_vec));
allan_ay = zeros(size(m_vec));
allan_az = zeros(size(m_vec));

for ii = 1:numel(m_vec)
    m = m_vec(ii);
    K = floor(n_kept / m);            % number of non-overlapping chunks

    % chunk means
    mx = mean(reshape(ax(1:K*m), m, K), 1);
    my = mean(reshape(ay(1:K*m), m, K), 1);
    mz = mean(reshape(az(1:K*m), m, K), 1);

    % Allan variance: 1/(2(K-1)) · Σ (ȳ[k+1] - ȳ[k])²
    allan_ax(ii) = sqrt(mean(diff(mx).^2) / 2);
    allan_ay(ii) = sqrt(mean(diff(my).^2) / 2);
    allan_az(ii) = sqrt(mean(diff(mz).^2) / 2);
end

% ─── Fit the slope-(+1/2) region on Z to extract K (rate random walk) ──
% Use τ ≥ 3 s.  Require at least 4 points — if not enough, fall back to
% a single-point estimate at the largest τ.
mask_rw  = tau_vec >= 3.0;
idx_rw   = find(mask_rw);

if numel(idx_rw) >= 4
    logt = log(tau_vec(idx_rw)).';
    logy = log(allan_az(idx_rw)).';
    p    = polyfit(logt, logy, 1);     % slope m, intercept c in log space
    % If the measured slope isn't close to +1/2, the long-τ region is
    % dominated by something else (bias instability / thermal drift).
    % We still compute K assuming +1/2 because the Kalman filter needs
    % *some* random-walk model — warn the user in the printout.
    slope_est = p(1);
    % σ(τ) = K·√(τ/3)  →  at τ=3, σ = K
    K_rw = interp1(tau_vec(idx_rw), allan_az(idx_rw), 3.0, 'linear','extrap');
else
    slope_est = NaN;
    K_rw = allan_az(end);              % fallback
    idx_rw = numel(tau_vec);
end

q_bias_recommended = K_rw^2 * DT_S;

% ─── Print summary ─────────────────────────────────────────────────────
fprintf("\n--- Bench statistics ---\n");
fprintf("  mean (az)  = %+8.5f m/s²\n",  mean(az));
fprintf("  std  (az)  =  %8.5f m/s²\n",  std(az));

fprintf("\n--- Allan deviation (Z-axis) ---\n");
for ii = [1, round(numel(tau_vec)/3), round(2*numel(tau_vec)/3), numel(tau_vec)]
    fprintf("  τ = %7.3f s    σ_A = %.4e m/s²\n", tau_vec(ii), allan_az(ii));
end

fprintf("\n--- Rate random walk fit (τ ≥ 3 s) ---\n");
if ~isnan(slope_est)
    fprintf("  log-log slope   = %+6.3f  (ideal: +0.500)\n", slope_est);
end
fprintf("  K (rate rand. walk) = %.4e m/s²/√s\n", K_rw);

fprintf("\n--- Copy into main.c ---\n");
fprintf("  q_bias = K^2 · dt = %.4e   (per-step bias variance)\n", ...
        q_bias_recommended);
fprintf("\n");
fprintf("static const float32_t KZ_Q[9] = {\n");
fprintf("    /* row 0:  h / {h, v, b} */\n");
fprintf("    %.4ef,   %.4ef,   0.0f,\n", 0.0, 1.9e-10);        % paste from accel_covariance.m
fprintf("    /* row 1:  v / {h, v, b} */\n");
fprintf("    %.4ef,   %.4ef,   0.0f,\n", 1.9e-10, 387.7e-10);  % paste from accel_covariance.m
fprintf("    /* row 2:  b / {h, v, b}  (this script) */\n");
fprintf("    0.0f,          0.0f,          %.4ef\n", q_bias_recommended);
fprintf("};\n");
fprintf("\nNOTE: the row 0/1 values above are placeholders — replace with\n");
fprintf("      σ_az² · {dt⁴/4, dt³/2, dt²} from accel_covariance.m.\n");

% ─── Plot ──────────────────────────────────────────────────────────────
figure('Name','Allan deviation — BNO085 linear acceleration');
loglog(tau_vec, allan_ax, 'r-o', 'DisplayName','a_x');  hold on;
loglog(tau_vec, allan_ay, 'g-s', 'DisplayName','a_y');
loglog(tau_vec, allan_az, 'b-^', 'DisplayName','a_z  (drives Kalman)');

% reference slope lines
tau_ref = [tau_vec(1), tau_vec(end)];
ref_m12 = allan_az(1) * sqrt(tau_vec(1)./tau_ref);
ref_p12 = K_rw     * sqrt(tau_ref./3);
loglog(tau_ref, ref_m12, 'k--', 'DisplayName','slope -½ (white noise)');
loglog(tau_ref, ref_p12, 'k:',  'DisplayName','slope +½ (random walk)');

grid on;  xlabel('\tau  [s]');  ylabel('Allan deviation  [m/s²]');
title('BNO085 SH2\_LINEAR\_ACCELERATION  —  Allan deviation');
legend('Location','best');

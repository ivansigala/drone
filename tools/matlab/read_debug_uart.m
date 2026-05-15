function data = read_raw_uart(comPort, baudRate, captureSeconds)
%READ_RAW_UART  Capture a fixed window of framed telemetry, then plot.
%
%   read_raw_uart('COM16')                      % prompts for capture seconds
%   read_raw_uart('COM16', 115200, 30)          % no prompt, 30 s capture
%   data = read_raw_uart('COM16', 115200, 30)   % same + return the data
%
% Wire format (must match debug_uart_send_frame() in firmware):
%
%   [SYNC1=0xAA][SYNC2=0x55][SEQ][ID][LEN=11][PAYLOAD][CRC8]
%
% PAYLOAD layout (little-endian on the wire) -- 15 bytes:
%
%   bytes 0..3    uint32   ts_ms        LPTMR0-derived timestamp [ms]
%   byte  4       uint8    motor_id     0..3
%   bytes 5..8    float32  omega_rad_s  measured mechanical angular velocity
%   bytes 9..12   float32  target_rad_s PID setpoint from ControlLoopTask
%   bytes 13..14  uint16   throttle     last commanded DSHOT throttle (0..2047)
%
% The plot uses ts_ms (device time) as the x-axis -- USB-CDC scheduling
% jitter no longer smears the time axis. After capture the per-motor
% data is auto-saved to a timestamped .mat file in the current folder.

if nargin < 2 || isempty(baudRate)
    baudRate = 115200;
end
if nargin < 3 || isempty(captureSeconds)
    s = input('Capture duration [seconds]: ', 's');
    captureSeconds = str2double(s);
    if isnan(captureSeconds) || captureSeconds <= 0
        error('read_raw_uart:badDuration', ...
              'Invalid capture duration -- must be a positive number.');
    end
end

% --- Constants ---------------------------------------------------------
SYNC1            = uint8(0xAA);
SYNC2            = uint8(0x55);
ID_TELEMETRY     = uint8(0x01);
NUM_MOTORS       = 4;
MAX_PAYLOAD      = 250;
TELEMETRY_LEN    = 15;
N_PER_MOTOR      = ceil(captureSeconds * 500) + 1024;

% --- Open the port -----------------------------------------------------
fprintf('Opening %s @ %d baud...\n', comPort, baudRate);
sp = serialport(comPort, baudRate, 'Timeout', 10);
flush(sp);
cleanupObj = onCleanup(@() delete(sp)); %#ok<NASGU>

% --- Preallocated per-motor capture buffers ----------------------------
t_buf        = zeros(N_PER_MOTOR, NUM_MOTORS);   % seconds (device time)
omega_buf    = zeros(N_PER_MOTOR, NUM_MOTORS);   % rad/s   (measured)
target_buf   = zeros(N_PER_MOTOR, NUM_MOTORS);   % rad/s   (PID setpoint)
throttle_buf = zeros(N_PER_MOTOR, NUM_MOTORS);   % 0..2047
counts       = zeros(NUM_MOTORS, 1);

% --- CRC8 lookup table -------------------------------------------------
crcTable = build_crc8_table(uint8(0x07));

% --- Streaming state machine -------------------------------------------
ST_FIND_S1 = 0; ST_FIND_S2 = 1;
ST_SEQ     = 2; ST_ID      = 3; ST_LEN  = 4;
ST_PAY     = 5; ST_CRC     = 6;

state    = ST_FIND_S1;
seq_b    = uint8(0);
id_b     = uint8(0);
lenN     = 0;
payload  = zeros(1, MAX_PAYLOAD, 'uint8');
payIdx   = 0;
crcCalc  = uint8(0);

% Stats
nGood    = 0;
nBadCrc  = 0;
nResync  = 0;
nDropped = 0;
nWrongLen= 0;
lastSeq  = -1;

% Reference time: device ts_ms of the very first valid frame is mapped
% to t = 0 s. Subsequent frames are plotted as (ts_ms - ts0) / 1000.
ts0_ms = -1;

fprintf('Capturing %.2f s of data...\n', captureSeconds);

% --- Capture loop (no plotting) ----------------------------------------
tStart    = tic;
tProgress = tic;

while toc(tStart) < captureSeconds
    nAvail = sp.NumBytesAvailable;
    if nAvail == 0
        pause(0.001);
    else
        chunk = read(sp, nAvail, 'uint8');

        for i = 1:numel(chunk)
            b = uint8(chunk(i));
            switch state
                case ST_FIND_S1
                    if b == SYNC1
                        state = ST_FIND_S2;
                    end
                case ST_FIND_S2
                    if b == SYNC2
                        state   = ST_SEQ;
                        crcCalc = uint8(0);
                        payIdx  = 0;
                    elseif b ~= SYNC1
                        state   = ST_FIND_S1;
                        nResync = nResync + 1;
                    end
                case ST_SEQ
                    seq_b   = b;
                    crcCalc = crcTable(double(bitxor(crcCalc, b)) + 1);
                    state   = ST_ID;
                case ST_ID
                    id_b    = b;
                    crcCalc = crcTable(double(bitxor(crcCalc, b)) + 1);
                    state   = ST_LEN;
                case ST_LEN
                    lenN    = double(b);
                    crcCalc = crcTable(double(bitxor(crcCalc, b)) + 1);
                    if lenN == 0
                        state = ST_CRC;
                    elseif lenN > MAX_PAYLOAD
                        state   = ST_FIND_S1;
                        nResync = nResync + 1;
                    else
                        state = ST_PAY;
                    end
                case ST_PAY
                    payIdx          = payIdx + 1;
                    payload(payIdx) = b;
                    crcCalc         = crcTable(double(bitxor(crcCalc, b)) + 1);
                    if payIdx >= lenN
                        state = ST_CRC;
                    end
                case ST_CRC
                    if b == crcCalc
                        nGood = nGood + 1;

                        % SEQ-skip detector
                        if lastSeq >= 0
                            expected = mod(lastSeq + 1, 256);
                            if double(seq_b) ~= expected
                                gap      = mod(double(seq_b) - expected, 256);
                                nDropped = nDropped + gap;
                            end
                        end
                        lastSeq = double(seq_b);

                        if id_b == ID_TELEMETRY
                            if lenN == TELEMETRY_LEN
                                % Decode the 15-byte payload.
                                ts_ms  = double(typecast( ...
                                    uint8(payload(1:4)), 'uint32'));
                                mid    = double(payload(5));
                                omega  = double(typecast( ...
                                    uint8(payload(6:9)), 'single'));
                                target = double(typecast( ...
                                    uint8(payload(10:13)), 'single'));
                                thr    = double(typecast( ...
                                    uint8(payload(14:15)), 'uint16'));

                                if mid >= 0 && mid < NUM_MOTORS
                                    if ts0_ms < 0
                                        ts0_ms = ts_ms;
                                    end
                                    col = mid + 1;
                                    ci  = counts(col) + 1;
                                    if ci <= N_PER_MOTOR
                                        t_buf(ci, col)        = (ts_ms - ts0_ms) / 1000.0;
                                        omega_buf(ci, col)    = omega;
                                        target_buf(ci, col)   = target;
                                        throttle_buf(ci, col) = thr;
                                        counts(col)           = ci;
                                    end
                                end
                            else
                                % Unexpected LEN for this ID -- usually means
                                % stale firmware vs. parser. Count and ignore.
                                nWrongLen = nWrongLen + 1;
                            end
                        end
                    else
                        nBadCrc = nBadCrc + 1;
                    end
                    state = ST_FIND_S1;
            end
        end
    end

    if toc(tProgress) > 1.0
        fprintf(['  %5.1f / %.1f s   ok=%d  crc=%d  resync=%d  ' ...
                 'drops=%d  wrongLen=%d\n'], ...
                toc(tStart), captureSeconds, ...
                nGood, nBadCrc, nResync, nDropped, nWrongLen);
        tProgress = tic;
    end
end

fprintf(['Capture done. ok=%d  crc=%d  resync=%d  drops=%d  ' ...
         'wrongLen=%d\n'], nGood, nBadCrc, nResync, nDropped, nWrongLen);
fprintf('Per-motor frame counts: %d  %d  %d  %d\n', ...
        counts(1), counts(2), counts(3), counts(4));

% --- Plot once at the end ----------------------------------------------
fig = figure('Name', sprintf('Motor telemetry @ %s (%.1f s)', ...
                             comPort, captureSeconds), ...
             'NumberTitle', 'off', 'Color', [0.08 0.08 0.08]);

% Top axes: omega [rad/s] over device time.
ax1 = subplot(2, 1, 1, 'Parent', fig);
set(ax1, 'Color', [0.12 0.12 0.12], 'XColor', [0.9 0.9 0.9], ...
    'YColor', [0.9 0.9 0.9], 'GridColor', [0.5 0.5 0.5], 'GridAlpha', 0.4);
hold(ax1, 'on'); grid(ax1, 'on');
ylabel(ax1, '\omega [rad/s]', 'Color', 'w');
title(ax1, sprintf( ...
        'Motor angular velocity | %.2f s | ok=%d  crc=%d  drops=%d', ...
        captureSeconds, nGood, nBadCrc, nDropped), ...
      'Color', 'w');

% Bottom axes: throttle command over device time.
ax2 = subplot(2, 1, 2, 'Parent', fig);
set(ax2, 'Color', [0.12 0.12 0.12], 'XColor', [0.9 0.9 0.9], ...
    'YColor', [0.9 0.9 0.9], 'GridColor', [0.5 0.5 0.5], 'GridAlpha', 0.4);
hold(ax2, 'on'); grid(ax2, 'on');
xlabel(ax2, 'Device time [s]', 'Color', 'w');
ylabel(ax2, 'Throttle [DSHOT 0..2047]', 'Color', 'w');

linkaxes([ax1, ax2], 'x');

neon_colors = [0   1   1;
               1   0   1;
               1   1   0;
               0.2 1   0.2];

for k = 1:NUM_MOTORS
    n = counts(k);
    if n == 0, continue; end
    colIdx = mod(k-1, size(neon_colors, 1)) + 1;

    % Measured omega -- solid neon line.
    plot(ax1, t_buf(1:n, k), omega_buf(1:n, k), '-', ...
         'LineWidth', 1.5, 'Color', neon_colors(colIdx,:), ...
         'DisplayName', sprintf('Motor %d \\omega (%d)', k-1, n));

    % PID setpoint -- dashed white-ish line over the measured trace so
    % step responses are obvious. Hidden from the legend if it's just
    % a single constant value (avoids 4 redundant entries).
    plot(ax1, t_buf(1:n, k), target_buf(1:n, k), '--', ...
         'LineWidth', 1.0, 'Color', neon_colors(colIdx,:) * 0.7 + 0.3, ...
         'DisplayName', sprintf('Motor %d target', k-1));

    plot(ax2, t_buf(1:n, k), throttle_buf(1:n, k), '-', ...
         'LineWidth', 1.2, 'Color', neon_colors(colIdx,:), ...
         'DisplayName', sprintf('Motor %d', k-1));
end

if any(counts > 0)
    lgd = legend(ax1, 'Location', 'northeastoutside');
    set(lgd, 'TextColor', 'w', 'Color', [0.15 0.15 0.15], ...
             'EdgeColor', [0.3 0.3 0.3]);
end

% --- Build labelled per-motor structs ----------------------------------
motor0 = pack_motor(0, t_buf, omega_buf, target_buf, throttle_buf, counts);
motor1 = pack_motor(1, t_buf, omega_buf, target_buf, throttle_buf, counts);
motor2 = pack_motor(2, t_buf, omega_buf, target_buf, throttle_buf, counts);
motor3 = pack_motor(3, t_buf, omega_buf, target_buf, throttle_buf, counts);

meta = struct( ...
    'comPort',         comPort, ...
    'baudRate',        baudRate, ...
    'captureSeconds',  captureSeconds, ...
    'nGood',           nGood, ...
    'nBadCrc',         nBadCrc, ...
    'nResync',         nResync, ...
    'nDropped',        nDropped, ...
    'nWrongLen',       nWrongLen, ...
    'capturedAt',      datestr(now, 'yyyy-mm-dd HH:MM:SS')); %#ok<TNOW1,DATST>

% --- Auto-save to a timestamped .mat file ------------------------------
fname = sprintf('motor_telemetry_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')); %#ok<TNOW1,DATST>
save(fname, 'motor0', 'motor1', 'motor2', 'motor3', 'meta');
fprintf('Saved %s\n', fname);

% --- Optional return value ---------------------------------------------
if nargout >= 1
    data.motor0 = motor0;
    data.motor1 = motor1;
    data.motor2 = motor2;
    data.motor3 = motor3;
    data.meta   = meta;
end
end % function read_raw_uart


%% ----------------------------------------------------------------------
% Pack a single motor's vectors into a labelled struct
%% ----------------------------------------------------------------------
function m = pack_motor(motor_id, t_buf, omega_buf, target_buf, throttle_buf, counts)
    col = motor_id + 1;
    n   = counts(col);
    m   = struct( ...
        'id',       motor_id,                  ... 0..3
        't',        t_buf(1:n, col),           ... seconds, device time
        'omega',    omega_buf(1:n, col),       ... rad/s   (measured)
        'target',   target_buf(1:n, col),      ... rad/s   (PID setpoint)
        'throttle', throttle_buf(1:n, col),    ... 0..2047
        'count',    n);
end


%% ----------------------------------------------------------------------
% CRC-8 / SMBus (poly 0x07, init 0x00) lookup-table builder
%% ----------------------------------------------------------------------
function tbl = build_crc8_table(poly)
    tbl = zeros(256, 1, 'uint8');
    for i = 0:255
        crc = uint8(i);
        for j = 1:8
            if bitand(crc, uint8(0x80))
                crc = bitxor(bitshift(crc, 1), poly);
            else
                crc = bitshift(crc, 1);
            end
            crc = uint8(bitand(crc, uint8(0xFF)));
        end
        tbl(i+1) = crc;
    end
end

% read_kalman_z.m
%
% Live plot of the Kalman-Z (vertical position + velocity) telemetry the
% drone firmware streams over LPUART4 -> USB-CDC.
%
% Wire protocol (debug_uart_mcxn947)
% ----------------------------------
%   [SYNC1=0xAA][SYNC2=0x55][SEQ][ID][LEN][PAYLOAD..LEN][CRC8]
%   CRC8 = SMBus (poly 0x07, init 0x00) over [SEQ, ID, LEN, PAYLOAD]
%
% This script keeps frame ID 0x04 (DEBUG_UART_FRAME_ID_KALMAN_Z) and
% discards everything else. The Kalman-Z payload is little-endian:
%
%   bytes 0..3   uint32   ts_ms          firmware LPTMR0 timestamp [ms]
%   bytes 4..7   float32  z_kalman_m     filtered altitude          [m]
%   bytes 8..11  float32  v_kalman_m_s   filtered vertical velocity [m/s]
%
% Usage
% -----
%   1. Flash the drone firmware and confirm the USB-CDC bridge enumerates
%      as COM9 (change PORT_NAME below if it doesn't).
%   2. >> read_kalman_z
%   3. Close the figure window to stop streaming.
%
% Author: Diego, 2026

clear; clc; close all;

% -------------------------------------------------------------------------
% User configuration
% -------------------------------------------------------------------------
PORT_NAME      = "COM9";
BAUDRATE       = 115200;        % matches DEBUG_UART_BAUDRATE in main.c
PLOT_WINDOW_S  = 20;            % rolling time window shown on the plot

% Protocol constants (do not change unless the firmware does)
SYNC1                       = uint8(hex2dec('AA'));
SYNC2                       = uint8(hex2dec('55'));
FRAME_ID_KALMAN_Z           = uint8(hex2dec('04'));
KALMAN_Z_PAYLOAD_LEN        = uint8(12);

% -------------------------------------------------------------------------
% Open the serial port
% -------------------------------------------------------------------------
fprintf('Opening %s @ %d baud...\n', PORT_NAME, BAUDRATE);
sp = serialport(PORT_NAME, BAUDRATE);
configureTerminator(sp, "LF");            % terminator is unused for binary frames
flush(sp);

% -------------------------------------------------------------------------
% Pre-compute the CRC-8 (SMBus, poly 0x07, init 0x00) lookup table.
% -------------------------------------------------------------------------
crcTable = zeros(256, 1, 'uint8');
for b = 0:255
    c = uint8(b);
    for i = 1:8
        if bitand(c, uint8(128)) ~= 0
            c = bitxor(bitshift(c, 1), uint8(hex2dec('07')));
        else
            c = bitshift(c, 1);
        end
    end
    crcTable(b+1) = c;
end

% -------------------------------------------------------------------------
% Rolling sample buffer + figure
% -------------------------------------------------------------------------
maxSamples = 50 * PLOT_WINDOW_S * 2;       % 50 Hz * window * generous safety
tBuf = nan(maxSamples, 1);
zBuf = nan(maxSamples, 1);
vBuf = nan(maxSamples, 1);
nSamples = 0;

fig = figure('Name', 'Kalman-Z live', 'NumberTitle', 'off', ...
             'Position', [200 200 900 600]);

axZ = subplot(2,1,1);
hZ  = animatedline('Color', [0.10 0.45 0.85], 'LineWidth', 1.2);
grid(axZ, 'on');
ylabel(axZ, 'Z position [m]');
title(axZ, sprintf('Kalman-Z telemetry (frame ID 0x%02X)', FRAME_ID_KALMAN_Z));

axV = subplot(2,1,2);
hV  = animatedline('Color', [0.85 0.30 0.15], 'LineWidth', 1.2);
grid(axV, 'on');
ylabel(axV, 'Vertical velocity [m/s]');
xlabel(axV, 'Time [s]');

linkaxes([axZ, axV], 'x');

% -------------------------------------------------------------------------
% Frame parser state machine
%
%   state = 0 : looking for SYNC1
%   state = 1 : got SYNC1, looking for SYNC2
%   state = 2 : reading SEQ
%   state = 3 : reading ID
%   state = 4 : reading LEN
%   state = 5 : reading PAYLOAD (LEN bytes)
%   state = 6 : reading CRC
% -------------------------------------------------------------------------
state    = 0;
seq      = uint8(0);
fid      = uint8(0);
flen     = uint8(0);
payload  = zeros(255, 1, 'uint8');
pidx     = 0;

framesOK   = 0;
framesCRC  = 0;
framesSeen = 0;

% -------------------------------------------------------------------------
% Callback wiring: parse incoming bytes whenever any arrive
% -------------------------------------------------------------------------
configureCallback(sp, "byte", 1, @(src, ~) onBytes(src));

fprintf('Streaming... close the figure to stop.\n');

% Keep the script alive while the figure is open
while ishandle(fig)
    pause(0.05);
end

% -------------------------------------------------------------------------
% Tear down
% -------------------------------------------------------------------------
fprintf('\n--- Session stats ---\n');
fprintf('  Kalman-Z frames OK   : %d\n', framesOK);
fprintf('  CRC failures         : %d\n', framesCRC);
fprintf('  Frames of other IDs  : %d\n', framesSeen - framesOK - framesCRC);

try %#ok<TRYNC>
    configureCallback(sp, "off");
end
clear sp;

% =========================================================================
%  Local helpers (nested-style — share workspace with the script above)
% =========================================================================
function onBytes(src)
    % Reach into the script's workspace via 'evalin'? No — use shared
    % via assignin pattern would be messy. Instead, pull state from the
    % base workspace where the script lives.

    chunk = read(src, src.NumBytesAvailable, "uint8");
    if isempty(chunk), return; end

    % Pull shared state
    state    = evalin('base', 'state');
    seq      = evalin('base', 'seq');
    fid      = evalin('base', 'fid');
    flen     = evalin('base', 'flen');
    payload  = evalin('base', 'payload');
    pidx     = evalin('base', 'pidx');

    SYNC1                = evalin('base', 'SYNC1');
    SYNC2                = evalin('base', 'SYNC2');
    FRAME_ID_KALMAN_Z    = evalin('base', 'FRAME_ID_KALMAN_Z');
    KALMAN_Z_PAYLOAD_LEN = evalin('base', 'KALMAN_Z_PAYLOAD_LEN');
    crcTable             = evalin('base', 'crcTable');
    PLOT_WINDOW_S        = evalin('base', 'PLOT_WINDOW_S');
    maxSamples           = evalin('base', 'maxSamples');

    tBuf       = evalin('base', 'tBuf');
    zBuf       = evalin('base', 'zBuf');
    vBuf       = evalin('base', 'vBuf');
    nSamples   = evalin('base', 'nSamples');

    hZ         = evalin('base', 'hZ');
    hV         = evalin('base', 'hV');
    axZ        = evalin('base', 'axZ');
    axV        = evalin('base', 'axV');

    framesOK   = evalin('base', 'framesOK');
    framesCRC  = evalin('base', 'framesCRC');
    framesSeen = evalin('base', 'framesSeen');

    plotDirty = false;

    for k = 1:length(chunk)
        b = uint8(chunk(k));

        switch state
            case 0
                if b == SYNC1, state = 1; end
            case 1
                if b == SYNC2
                    state = 2;
                else
                    % Re-arm on possible new SYNC1
                    state = (b == SYNC1);
                end
            case 2
                seq   = b;
                state = 3;
            case 3
                fid   = b;
                state = 4;
            case 4
                flen  = b;
                pidx  = 0;
                if flen == 0
                    state = 6;
                else
                    state = 5;
                end
            case 5
                pidx = pidx + 1;
                payload(pidx) = b;
                if pidx >= flen
                    state = 6;
                end
            case 6
                framesSeen = framesSeen + 1;

                % CRC over [SEQ, ID, LEN, PAYLOAD]
                crc = uint8(0);
                hdr = uint8([seq, fid, flen]);
                for ii = 1:3
                    crc = crcTable(bitxor(crc, hdr(ii)) + 1);
                end
                for ii = 1:flen
                    crc = crcTable(bitxor(crc, payload(ii)) + 1);
                end

                if crc == b
                    if fid == FRAME_ID_KALMAN_Z && flen == KALMAN_Z_PAYLOAD_LEN
                        % Unpack little-endian: u32 ts_ms, f32 z, f32 v
                        ts_ms = typecast(uint8(payload(1:4)),  'uint32');
                        z_m   = typecast(uint8(payload(5:8)),  'single');
                        v_m_s = typecast(uint8(payload(9:12)), 'single');

                        nSamples = nSamples + 1;
                        idx = mod(nSamples - 1, maxSamples) + 1;
                        tBuf(idx) = double(ts_ms) / 1000.0;
                        zBuf(idx) = double(z_m);
                        vBuf(idx) = double(v_m_s);

                        addpoints(hZ, tBuf(idx), zBuf(idx));
                        addpoints(hV, tBuf(idx), vBuf(idx));
                        plotDirty = true;

                        framesOK = framesOK + 1;
                    end
                else
                    framesCRC = framesCRC + 1;
                end

                state = 0;
            otherwise
                state = 0;
        end
    end

    if plotDirty
        % Roll the x-axis to keep PLOT_WINDOW_S of history visible
        tNow = tBuf(mod(nSamples - 1, maxSamples) + 1);
        if isfinite(tNow)
            xlim(axZ, [max(0, tNow - PLOT_WINDOW_S), tNow + 0.5]);
            xlim(axV, [max(0, tNow - PLOT_WINDOW_S), tNow + 0.5]);
        end
        drawnow limitrate;
    end

    % Push state back to base workspace
    assignin('base', 'state',      state);
    assignin('base', 'seq',        seq);
    assignin('base', 'fid',        fid);
    assignin('base', 'flen',       flen);
    assignin('base', 'payload',    payload);
    assignin('base', 'pidx',       pidx);
    assignin('base', 'tBuf',       tBuf);
    assignin('base', 'zBuf',       zBuf);
    assignin('base', 'vBuf',       vBuf);
    assignin('base', 'nSamples',   nSamples);
    assignin('base', 'framesOK',   framesOK);
    assignin('base', 'framesCRC',  framesCRC);
    assignin('base', 'framesSeen', framesSeen);
end

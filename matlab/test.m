% hil_simulation.m
% HIL Plant Model for Drone SMC Controller
clear; clc; close all;

%% 1. Serial Port Setup
port = "COM16";
baudrate = 115200;          % MCU is configured to match this
s = serialport(port, baudrate, "Timeout", 20);
configureTerminator(s, "CR/LF"); % printf uses \r\n
flush(s);                        % drop any stale bytes from previous run

%% 2. Drone Parameters (matches Simulink Callback)
T   = 0.04;                 % 20 ms control period (50 Hz)
g   = 9.81;

M       = [0.05609; 0.30355; 0.04598; 0.04598; 0.0371; 0.0371; 0.0371; 0.0371];
M_tot   = sum(M);

% Aerodynamic drag
Sx = 0.0024; Sy = 0.0072; Sz = 0.0072;

% Motor positions — DIAGONAL pairing, matches the Callback exactly.
x_g  = 0.25301/2;
y_g  = 0.18625/2;
x1_g =  x_g;   y1_g =  y_g;
x2_g = -x_g;   y2_g = -y_g;
x3_g = -x_g;   y3_g =  y_g;
x4_g =  x_g;   y4_g = -y_g;

Cm  = 3.5e-6;
k1  = 0.7426e-6;
k2  = 0.1485e-6;

% Body-frame inertia (matches firmware DRONE_J_DATA)
I = [0.0038, 0.0001, 0;
     0.0001, 0.0056, 0;
     0,      0,      0.0055];

% Mixer:   U = Mot * (sign(w).*w.^2)   =>   w_sq = Mot \ U
Mot = [ k2,        k2,        k2,        k2;
        k2*y1_g,   k2*y2_g,   k2*y3_g,   k2*y4_g;
       -k2*x1_g,  -k2*x2_g,  -k2*x3_g,  -k2*x4_g;
       -k1,       -k1,        k1,        k1];

rcond_Mot = rcond(Mot);
fprintf('rcond(Mot) = %.3e\n', rcond_Mot);
if rcond_Mot < 1e-10
    error('Mixer matrix is singular. Check motor coordinates.');
end

%% 3. Simulation Initialization
states = zeros(12, 1);
w      = zeros(4, 1);
sim_duration  = 10;                 % seconds (bump higher for a longer trace)
steps         = sim_duration / T;

log_states = zeros(12, steps);
log_U      = zeros(4,  steps);
log_w      = zeros(4,  steps);
log_time   = zeros(1,  steps); % This will now hold the MCU timestamps

disp('Starting HIL Simulation...');

for k = 1:steps
    % --- 1. Send 12 states + 4 motor speeds to MCU as an ASCII string ---
    tx_data = [states; w];
    tx_str = sprintf('%.6f ', tx_data); 
    writeline(s, tx_str); 

    % --- 2. Read 5 values back (Timestamp, U0, U1, U2, U3) ---
    rx_str = readline(s);
    
    if isempty(rx_str) || strlength(rx_str) == 0
        warning('HIL: MCU timed out at k=%d. Aborting.', k);
        steps = k - 1;                   % truncate log to what we got
        break;
    end
    
    % Parse the 5 floats from the string
    rx_data = sscanf(rx_str, '%f %f %f %f %f');
    
    if numel(rx_data) ~= 5
        warning('HIL: MCU returned malformed data "%s" at k=%d. Aborting.', rx_str, k);
        steps = k - 1;
        break;
    end
    
    % Extract Timestamp and Control Efforts
    mcu_timestamp = rx_data(1);
    U             = double(rx_data(2:5));              
    
    log_time(k) = mcu_timestamp;
    log_U(:, k) = U;

    % --- 3. Mixer: invert U -> sign(w).*w.^2, then recover w ---
    w_sq_signed = Mot \ U;               
    w_sq        = max(w_sq_signed, 0);   % real motors can't generate negative thrust
                                         
    w           = sqrt(w_sq);
    log_w(:, k) = w;

    % --- 4. Plant discrete step (forward Euler, matches Simulink blocks) ---
    roll   = states(7);  pitch  = states(8);  yaw   = states(9);
    droll  = states(10); dpitch = states(11); dyaw  = states(12);

    Rotn = [cos(yaw)*cos(pitch),  cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll),  sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll);
            sin(yaw)*cos(pitch),  cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll),  sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);
           -sin(pitch),           cos(pitch)*sin(roll),                                cos(pitch)*cos(roll)];

    Lambda = [1,  0,           -sin(pitch);
              0,  cos(roll),    sin(roll)*cos(pitch);
              0, -sin(roll),    cos(roll)*cos(pitch)];

    dLambda = [0,  0,                 -dpitch*cos(pitch);
               0, -droll*sin(roll),    droll*cos(pitch)*cos(roll) - dpitch*sin(pitch)*sin(roll);
               0, -droll*cos(roll),   -droll*cos(pitch)*sin(roll) - dpitch*cos(roll)*sin(pitch)];

    % Translational update
    S = [Sx*sign(states(4))*states(4)^2;
         Sy*sign(states(5))*states(5)^2;
         Sz*sign(states(6))*states(6)^2];

    tra_kp1 = states(4:6);
    tra_kp2 = T * ((1/M_tot) * (Rotn * [0; 0; U(1)] - [0; 0; M_tot*g] - S)) + tra_kp1;

    states(1:3) = T * states(4:6) + states(1:3);
    states(4:6) = tra_kp2;

    % Rotational update
    n_kp1     = states(10:12);
    J         = Lambda' * I * Lambda;
    Omega     = Lambda * n_kp1;
    gyroRotor = [0; 0; Cm * sum(w)];
    tau       = U(2:4);

    n_kp2 = (J*Lambda) \ (tau - J*dLambda*n_kp1 - cross(Omega, J*Omega) - cross(Omega, gyroRotor));
    n_kp2 = n_kp2 * T + n_kp1;

    states(7:9)   = T * states(10:12) + states(7:9);
    states(10:12) = n_kp2;

    log_states(:, k) = states;
end

disp('Simulation Complete!');
clear s;

%% 4. Plot Results
if steps > 0
    valid = 1:steps;
    
    figure('Name', 'Position (HIL)');
    subplot(3,1,1); plot(log_time(valid), log_states(1,valid)); title('X Position'); ylabel('m'); grid on;
    subplot(3,1,2); plot(log_time(valid), log_states(2,valid)); title('Y Position'); ylabel('m'); grid on;
    subplot(3,1,3); plot(log_time(valid), log_states(3,valid)); title('Z Position'); ylabel('m'); xlabel('MCU Time (s)'); grid on;

    figure('Name', 'Attitude (HIL)');
    subplot(3,1,1); plot(log_time(valid), log_states(7,valid)*180/pi);  title('Roll');  ylabel('deg'); grid on;
    subplot(3,1,2); plot(log_time(valid), log_states(8,valid)*180/pi);  title('Pitch'); ylabel('deg'); grid on;
    subplot(3,1,3); plot(log_time(valid), log_states(9,valid)*180/pi);  title('Yaw');   ylabel('deg'); xlabel('MCU Time (s)'); grid on;
    
    figure('Name', 'Control Effort (HIL)');
    subplot(4,1,1); plot(log_time(valid), log_U(1,valid)); title('U0 (Total Thrust)'); ylabel('N'); grid on;
    subplot(4,1,2); plot(log_time(valid), log_U(2,valid)); title('U1 (Roll Torque)'); ylabel('N·m'); grid on;
    subplot(4,1,3); plot(log_time(valid), log_U(3,valid)); title('U2 (Pitch Torque)'); ylabel('N·m'); grid on;
    subplot(4,1,4); plot(log_time(valid), log_U(4,valid)); title('U3 (Yaw Torque)'); ylabel('N·m'); xlabel('MCU Time (s)'); grid on;
end
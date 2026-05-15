% hil_simulation.m
% HIL Plant Model for Drone SMC Controller
clear; clc; close all;

%% 1. Serial Port Setup
port = "COM16";
baudrate = 115200; 
s = serialport(port, baudrate, "Timeout", 2);
flush(s); % Clear any junk in the buffer before starting

%% 2. Drone Parameters
T = 0.1; % 10 ms period
g = 9.81;

M = [0.05609; 0.30355; 0.04598; 0.04598; 0.0371; 0.0371; 0.0371; 0.0371];
M_tot = sum(M);

% Coordinates & Aerodynamics
Sx = 0.0024; Sy = 0.0072; Sz = 0.0072;
y_g = 0.18625/2; x_g = 0.25301/2;
x1_g = x_g;  y1_g = y_g;
x2_g = -x_g; y2_g = y_g;
x3_g = -x_g; y3_g = -y_g;
x4_g = x_g;  y4_g = -y_g;

Cm = 0.0000035;
k1 = 0.7426e-6; 
k2 = 0.1485e-6; 

I = [0.0038, 0.0001, 0; 
     0.0001, 0.0056, 0; 
     0,      0,      0.0055];

% FIXED MIXER MATRIX: Alternating signs for Yaw (Row 4) to make it invertible
Mot = [ k2,       k2,       k2,       k2;
        k2*y1_g,  k2*y2_g,  k2*y3_g,  k2*y4_g;
       -k2*x1_g, -k2*x2_g, -k2*x3_g, -k2*x4_g;
       -k1,       k1,      -k1,       k1]; % <--- Fixed Row
       
Mot_inv = inv(Mot);

%% 3. Simulation Initialization
states = zeros(12, 1);
w      = [0; 0; 0; 0]; 
U      = [0; 0; 0; 0]; % Initialize U to prevent crashes if first read drops

time = 0;
sim_duration = 10; 
steps = sim_duration / T;

log_states = zeros(12, steps);
log_time = zeros(1, steps);

disp('Starting HIL Simulation...');

for k = 1:steps
    % 1. Send data to MCU (64 bytes)
    tx_data = single([states; w]);
    write(s, tx_data, "single");
    
    % 2. Receive control effort from MCU
    rx_data = read(s, 4, "single");
    
    % SAFETY CHECK: Did we timeout?
    if isempty(rx_data) || length(rx_data) < 4
        disp(['Warning: Serial timeout at step ', num2str(k), '. Re-using previous U.']);
        flush(s); % Clear buffer to resync
    else
        U = double(rx_data)'; 
    end
    
    % 3. Calculate motor speeds
    w_sq = Mot_inv * U;
    w_sq = max(w_sq, 0); 
    w = sqrt(w_sq);
    % 4. Plant Discrete Step (Forward Euler / Simulink logic)
    roll = states(7); pitch = states(8); yaw = states(9);
    droll = states(10); dpitch = states(11); dyaw = states(12);
    
    % Rotation & Lambda Matrices
    Rotn = [cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll), sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll);
            sin(yaw)*cos(pitch), cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll), sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);
            -sin(pitch),         cos(pitch)*sin(roll),                               cos(pitch)*cos(roll)];
            
    Lambda = [1, 0, -sin(pitch);
              0, cos(roll), sin(roll)*cos(pitch);
              0, -sin(roll), cos(roll)*cos(pitch)];
              
    dLambda = [0, 0, -dpitch*cos(pitch);
               0, -droll*sin(roll), droll*cos(pitch)*cos(roll) - dpitch*sin(pitch)*sin(roll);
               0, -droll*cos(roll), -droll*cos(pitch)*sin(roll) - dpitch*cos(roll)*sin(pitch)];

    % Translational Dynamics Update
    S = [Sx*sign(states(4))*(states(4)^2); 
         Sy*sign(states(5))*(states(5)^2); 
         Sz*sign(states(6))*(states(6)^2)];
         
    tra_kp1 = states(4:6);
    tra_kp2 = T * ((1/M_tot) * (Rotn * [0; 0; U(1)] - [0; 0; M_tot*g] - S)) + tra_kp1;
    
    states(1:3) = T * states(4:6) + states(1:3); % x, y, z update
    states(4:6) = tra_kp2;                       % dx, dy, dz update
    
    % Rotational Dynamics Update
    n_kp1 = states(10:12);
    J = Lambda' * I * Lambda;
    Omega = Lambda * n_kp1;
    gyroRotor = [0; 0; Cm * sum(w)];
    tau = U(2:4);
    
    n_kp2 = ((J * Lambda) \ (tau - J * dLambda * n_kp1 - cross(Omega, J * Omega) - cross(Omega, gyroRotor))) * T + n_kp1;
    
    states(7:9) = T * states(10:12) + states(7:9); % roll, pitch, yaw update
    states(10:12) = n_kp2;                         % droll, dpitch, dyaw update
    
    % Log Data
    log_states(:, k) = states;
    log_time(k) = time;
    time = time + T;
end

disp('Simulation Complete!');
clear s; % Release serial port

%% 4. Plot Results
figure;
subplot(3,1,1); plot(log_time, log_states(1,:)); title('X Position'); ylabel('m'); grid on;
subplot(3,1,2); plot(log_time, log_states(2,:)); title('Y Position'); ylabel('m'); grid on;
subplot(3,1,3); plot(log_time, log_states(3,:)); title('Z Position'); ylabel('m'); xlabel('Time (s)'); grid on;
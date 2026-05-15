s = serialport("COM16", 115200, "Timeout", 5);
write(s, single([zeros(1,12), zeros(1,4)]), "single");
u = read(s, 4, "single");
disp(u)        % expect [5.886, ~0, ~0, ~0]  -- hover thrust + small yaw torque
clear s
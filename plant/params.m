function p = params()
    % Define the parameters of the plant
    p.m = 1.0; % Mass of the plant (kg)
    p.I = diag([0.01, 0.01, 0.02]); % Inertia matrix (kg*m^2)
    p.g = 9.81; % Gravitational acceleration (m/s^2)
    p.motor.tau = 0.1; % Motor time constant (s)
    p.motor.k = 0.01; % Motor gain (N*m/V)
    p.motor.max_thrust = 10; % Maximum thrust (N)
    p.motor.min_thrust = 0; % Minimum thrust (N)
    p.motor.delay = 0.05; % Motor delay (s)
    p.motor.torque_coeff = 0.02; % Reaction (yaw) torque per unit thrust (m)
    p.frame.arm_length = 0.15; % Center of mass to motor distance (m)
end
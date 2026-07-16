function [pdot, vdot, qdot, omegadot] = plant(r, v, q, omega, T)
    arguments
        r (1, 3) double
        v (1, 3) double
        q (1, 4) double
        omega (1, 3) double
        T (1, 4) double % Thrust vector
    end





    % Placeholder for plant dynamics calculation
    pdot = v; % Position derivative is velocity
    vdot = [0; 0; -9.81]/p.m; % Simple gravity effect
    qdot = 0.5 * quatmultiply(q, [0, omega]); % Quaternion derivative
    omegadot = [0; 0; 0]; % Placeholder for angular acceleration
end
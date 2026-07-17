function [rdot, vdot, qdot, omegadot] = plant(r, v, q, omega, T, p)
    arguments
        r (1, 3) double
        v (1, 3) double
        q (1, 4) double
        omega (1, 3) double
        T (1, 4) double % Thrust vector
        p (1, 1) struct
    end

    % Placeholder for plant dynamics calculation

    % Rotor thrust to total thrust
    T_total = sum(T); % Total thrust magnitude

    % Thrust to roll, pitch, yaw torques (assuming simple model)
    


    rdot = v; %  Position derivative is velocity
    F_thrust_world = quat_rotate(q, [0, 0, T_total]); % Rotate thrust to world frame
    F_gravity = [0, 0, -p.m * p.g]; % Gravity force in world frame
    vdot = (1/p.m) * (F_thrust_world + F_gravity); % Simple gravity effect
    qdot = quat_derivative(q, omega); % Quaternion derivative

    A = motor_mixing_matrix(p); % Get the motor mixing matrix

    wrench = A * T'; % Compute wrench from thrust vector
    tau_net = wrench(2:4)'; % Extract net torque from wrench, back to (1,3)
    Iw = (p.I * omega')';        % angular momentum L = I*omega, back to (1,3)
    gyro = cross(omega, Iw);      % both (1,3) — cross() wants matching orientation
    omegadot = (p.I \ (tau_net - gyro)')';  % solve I*x = tau_net - gyro as a column, transpose result back
end
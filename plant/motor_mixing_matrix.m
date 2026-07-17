function A = motor_mixing_matrix(p)
    % Per-motor thrust -> body wrench allocation matrix. Source of truth
    % for motor/frame convention -- consumers (control_allocator.m, etc.)
    % should reference this comment, not restate it.
    %
    % X configuration, PX4 quad X numbering/spin convention:
    %   motor 1 = front-right (CW), 2 = rear-left (CW),
    %   3 = front-left (CCW), 4 = rear-right (CCW).
    %   Body frame: x-forward, y-right, z-up, right-handed.
    %
    % A * [T1;T2;T3;T4]' = [F; tau_x; tau_y; tau_z], F along body +z.
        arguments
            p (1, 1) struct
        end
    L = p.frame.arm_length;
    k = p.motor.torque_coeff;
    s = sqrt(2) / 2;

    x_pos = L * s * [1, -1, 1, -1];
    y_pos = L * s * [1, -1, -1, 1];
    yaw_sign = [1, 1, -1, -1]; % CW motors add +yaw reaction torque, CCW subtract

    % Allocation matrix: per-motor thrust [T1;T2;T3;T4] -> [F; tau_x; tau_y; tau_z]
    A = [ones(1, 4);
            y_pos;
            -x_pos;
            k * yaw_sign];
    
end
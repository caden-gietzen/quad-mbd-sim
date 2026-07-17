function A = motor_mixing_matrix(p)
    % Per-motor thrust -> body wrench allocation matrix. Source of truth
    % for motor/frame convention -- consumers (control_allocator.m, etc.)
    % should reference this comment, not restate it.
    %
    % X configuration, PX4 quad X numbering/spin convention:
    %   motor 1 = front-right (CW), 2 = rear-left (CW),
    %   3 = front-left (CCW), 4 = rear-right (CCW).
    %   Body frame: FRD -- x-forward, y-right, z-down, right-handed.
    %
    % A * [T1;T2;T3;T4]' = [F; tau_x; tau_y; tau_z]. F is total thrust
    % magnitude (plant() applies it along body -z, i.e. up). The torque rows
    % come from tau = r x F with the thrust force along body -z:
    %   F_i = [0,0,-T_i] at r_i = [x_i,y_i,0]  ->  tau_i = [-y_i*T_i, x_i*T_i, 0]
    % hence tau_x = -y_pos, tau_y = +x_pos. The yaw reaction row also flips
    % sign versus a z-up build (a CW prop yaws the body about -z in FRD).
        arguments
            p (1, 1) struct
        end
    L = p.frame.arm_length;
    k = p.motor.torque_coeff;
    s = sqrt(2) / 2;

    x_pos = L * s * [1, -1, 1, -1];
    y_pos = L * s * [1, -1, -1, 1];
    yaw_sign = [1, 1, -1, -1]; % CW motors (1,2) yaw the body about -z in FRD; CCW (3,4) about +z

    % Allocation matrix: per-motor thrust [T1;T2;T3;T4] -> [F; tau_x; tau_y; tau_z]
    A = [ones(1, 4);
           -y_pos;         % tau_x = -y_pos   (r x F, thrust along -z)
            x_pos;         % tau_y = +x_pos
           -k * yaw_sign]; % yaw reaction, sign-flipped for FRD (z-down)
    
end
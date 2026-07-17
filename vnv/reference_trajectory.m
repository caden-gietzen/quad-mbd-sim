function [r_ref, v_ref, q_ref, omega_ref] = reference_trajectory(traj_type, t, p_traj)
    % Reference trajectory generator for open-loop plant
    % testing and closed-loop controller/Monte Carlo V&V scenarios.
    %
    % traj_type selects the scenario (integer, not string -- keeps this
    % codegen-safe if it ends up wired into a MATLAB Function block, same
    % constraint as everything else in this repo):
    %   0 = hold      -- hover at a fixed position/attitude
    %   1 = step      -- attitude step input (transient-response test)
    %   2 = circle    -- constant-altitude circle, heading tangent to travel
    %   3 = flip      -- full 360 deg rotation about one body axis
    %   4 = doublet   -- positive/negative step pair (system-ID excitation
    %                    input for sysid/'s two-stage ARX)
    %
    % Outputs match plant()'s state layout: r_ref/v_ref (1,3), q_ref (1,4)
    % scalar-first [w x y z], omega_ref (1,3). Position frame is NED (+z down),
    % consistent with plant()'s gravity term.
    arguments
        traj_type (1, 1) double
        t (1, 1) double
        p_traj (1, 1) struct
    end

    switch traj_type
        case 1
            [r_ref, v_ref, q_ref, omega_ref] = traj_step(t, p_traj);
        case 2
            [r_ref, v_ref, q_ref, omega_ref] = traj_circle(t, p_traj);
        case 3
            [r_ref, v_ref, q_ref, omega_ref] = traj_flip(t, p_traj);
        case 4
            [r_ref, v_ref, q_ref, omega_ref] = traj_doublet(t, p_traj);
        otherwise
            [r_ref, v_ref, q_ref, omega_ref] = traj_hold(t, p_traj);
    end
end

function [r_ref, v_ref, q_ref, omega_ref] = traj_hold(t, p_traj)
    % Hold a fixed position and attitude (hover / station-keeping).
    r_ref = p_traj.hold_position;
    v_ref = [0, 0, 0];
    q_ref = euler_to_quat([0, 0, p_traj.hold_yaw]);
    omega_ref = [0, 0, 0];
end

function [r_ref, v_ref, q_ref, omega_ref] = traj_step(t, p_traj)
    % Attitude step: hold hover position, step one Euler angle at
    % p_traj.step_time. Classic transient-response characterization input.
    r_ref = p_traj.hold_position;
    v_ref = [0, 0, 0];

    euler_ref = [0, 0, 0];
    if t >= p_traj.step_time
        euler_ref(p_traj.step_axis) = p_traj.step_amplitude;
    end
    q_ref = euler_to_quat(euler_ref);
    omega_ref = [0, 0, 0];
end

function [r_ref, v_ref, q_ref, omega_ref] = traj_circle(t, p_traj)
    % Constant-altitude circle, heading tangent to the direction of travel.
    w = p_traj.circle_rate;
    R = p_traj.circle_radius;

    x = R * cos(w * t);
    y = R * sin(w * t);
    z = -p_traj.circle_altitude; % NED: down-coordinate = -altitude
    vx = -R * w * sin(w * t);
    vy = R * w * cos(w * t);
    vz = 0;

    r_ref = [x, y, z];
    v_ref = [vx, vy, vz];
    yaw = atan2(vy, vx);
    q_ref = euler_to_quat([0, 0, yaw]);
    omega_ref = [0, 0, w];
end

function [r_ref, v_ref, q_ref, omega_ref] = traj_flip(t, p_traj)
    % Full 360 deg rotation about one body axis, position held at hover.
    % Exercises the full large-angle attitude envelope -- this is the
    % scenario the exact quat_to_rotvec log map (not the small-angle
    % approximation) is meant for.
    r_ref = p_traj.hold_position;
    v_ref = [0, 0, 0];

    if t >= p_traj.flip_start && t < p_traj.flip_start + p_traj.flip_duration
        angle = 2 * pi * (t - p_traj.flip_start) / p_traj.flip_duration;
        rate = 2 * pi / p_traj.flip_duration;
    else
        angle = 0;
        rate = 0;
    end

    axis_vec = [0, 0, 0];
    axis_vec(p_traj.flip_axis) = 1;
    q_ref = quat_normalize([cos(angle / 2), axis_vec * sin(angle / 2)]);
    omega_ref = axis_vec * rate;
end

function [r_ref, v_ref, q_ref, omega_ref] = traj_doublet(t, p_traj)
    % Positive/negative step pair -- standard flight-test excitation input
    % for system identification (sysid/'s two-stage ARX wants persistent
    % excitation, not a single step).
    r_ref = p_traj.hold_position;
    v_ref = [0, 0, 0];

    euler_ref = [0, 0, 0];
    t_rel = t - p_traj.doublet_start;
    if t_rel >= 0 && t_rel < p_traj.doublet_half_period
        euler_ref(p_traj.doublet_axis) = p_traj.doublet_amplitude;
    elseif t_rel >= p_traj.doublet_half_period && t_rel < 2 * p_traj.doublet_half_period
        euler_ref(p_traj.doublet_axis) = -p_traj.doublet_amplitude;
    end

    q_ref = euler_to_quat(euler_ref);
    omega_ref = [0, 0, 0];
end

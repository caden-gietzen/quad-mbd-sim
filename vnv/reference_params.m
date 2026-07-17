function p_traj = reference_params()
    % Default parameters for reference_trajectory().

    p_traj.hold_position = [0, 0, -2]; % NED [N E D] m; D=-2 is 2 m altitude. Hover
                                       % point + hold point for step/flip/doublet.
    p_traj.hold_yaw = 0; % rad

    p_traj.step_axis = 1; % 1=roll 2=pitch 3=yaw
    p_traj.step_amplitude = deg2rad(15); % rad
    p_traj.step_time = 1.0; % s

    p_traj.circle_radius = 2.0; % m
    p_traj.circle_rate = 0.5; % rad/s
    p_traj.circle_altitude = 2.0; % height above datum (m); traj_circle uses z = -altitude (NED)

    p_traj.flip_axis = 1; % 1=roll 2=pitch
    p_traj.flip_start = 1.0; % s
    p_traj.flip_duration = 1.0; % s, time to complete a full 360 deg rotation

    p_traj.doublet_axis = 2; % 1=roll 2=pitch 3=yaw
    p_traj.doublet_amplitude = deg2rad(10); % rad
    p_traj.doublet_start = 1.0; % s
    p_traj.doublet_half_period = 0.5; % s, duration of each half of the doublet
end

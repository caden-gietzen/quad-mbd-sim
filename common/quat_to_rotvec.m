function rotvec = quat_to_rotvec(q)
    % Convert a scalar-first [w x y z] quaternion to a 3-element rotation
    % vector (axis * angle, radians) via the exact log map -- the 3-DOF
    % attitude error signal a controller or EKF error state actually
    % consumes.
    %
    % Enforces the shortest-path sign convention (w >= 0) before
    % extracting the vector part: q and -q represent the identical
    % physical rotation, but correspond to angles near 0 and near 2*pi
    % respectively. Skipping this correction is the classic quaternion
    % attitude-control "unwinding" bug -- commanding the long way around
    % for what should be a small correction.
    if q(1) < 0
        q = -q;
    end

    v = q(2:4);
    v_norm = norm(v);
    theta = 2 * atan2(v_norm, q(1));

    if v_norm > 1e-8
        axis = v / v_norm;
    else
        axis = [0, 0, 0];
    end

    rotvec = theta * axis;
end

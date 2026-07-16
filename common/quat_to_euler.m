function euler_angles = quat_to_euler(q)
    % Convert quaternion to Euler angles (roll, pitch, yaw)
    % q is a 4-element vector [w, x, y, z]
    
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
    
    % Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x^2 + y^2);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    % Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi / 2; % use 90 degrees if out of range
    else
        pitch = asin(sinp);
    end
    
    % Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y^2 + z^2);
    yaw = atan2(siny_cosp, cosy_cosp);
    
    euler_angles = [roll, pitch, yaw];
end
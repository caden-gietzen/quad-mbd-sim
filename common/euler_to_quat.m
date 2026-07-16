function q = euler_to_quat(euler_angles)
    % Convert Euler angles (roll, pitch, yaw) to quaternion
    % euler_angles is a 3-element vector [roll, pitch, yaw]
    
    roll = euler_angles(1);
    pitch = euler_angles(2);
    yaw = euler_angles(3);
    
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);
    
    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
    
    q = [qw, qx, qy, qz];
end
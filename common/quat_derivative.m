function qdot = quat_derivative(q, omega)
    % Calculate the derivative of a quaternion given angular velocity
    qdot = 0.5 * quat_multiply(q, [0, omega]);
end
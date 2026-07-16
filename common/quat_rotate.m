function v = quat_rotate(q, v)
    % Rotate vector v by quaternion q
    qv = [0, v];
    q_conj = quat_conjugate(q);
    v_rotated = quat_multiply(quat_multiply(q, qv), q_conj);
    v = v_rotated(2:4); % Return the rotated vector part
end
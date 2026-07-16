function q_err = quat_error(q_ref, q)
    % Attitude error quaternion: the rotation from q_ref to q,
    % scalar-first [w x y z]. Identity ([1 0 0 0]) when q == q_ref.
    q_err = quat_multiply(quat_conjugate(q_ref), q);
end

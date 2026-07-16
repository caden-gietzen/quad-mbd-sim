function q = quat_conjugate(q)
    % Scalar-first [w x y z] quaternion conjugate
    q(2:4) = -q(2:4);
end
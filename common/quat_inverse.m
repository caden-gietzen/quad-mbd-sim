function q = quat_inverse(q)
    % Scalar-first [w x y z] quaternion inverse
    q = quat_conjugate(q) / (q * q');
end
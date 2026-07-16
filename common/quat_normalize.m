function q = quat_normalize(q)
    % Scalar-first [w x y z] quaternion normalize
    q = q / norm(q);
end
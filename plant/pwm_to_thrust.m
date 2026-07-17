function T = pwm_to_thrust(u, p)
    % Static PWM->thrust map, applied elementwise across the four motors.
    % Linear, normalized 0-1 PWM -- inverse of control/thrust_to_pwm.m;
    % keep the two in sync if this map ever changes.
    arguments
        u (1, 4) double
        p (1, 1) struct
    end

    T = p.motor.min_thrust + (p.motor.max_thrust - p.motor.min_thrust) * u;
end

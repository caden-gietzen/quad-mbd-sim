classdef tPlant < matlab.unittest.TestCase
    % Open-loop physics-verification suite for plant_test.slx.
    %
    % Run with:  runtests('tPlant')     % this file
    %       or:  runtests('vnv')        % the whole folder
    %
    % Each test drives the plant with a scripted PWM input via
    % run_plant_case and checks a physical property numerically, so a
    % regression fails a test instead of requiring a human to read a scope.
    %
    % Preconditions on plant_test.slx (see run_plant_case / unpack_state):
    %   - integrator ICs reference x0_r / x0_v / x0_q / x0_omega
    %   - the state Outport logs to yout, in unpack_state's column order.

    methods (Test)
        %% REQ-PLANT-001: ballistic trajectory (zero thrust, gravity only)

        function freeFall(tc)
            % Verifies REQ-PLANT-001 (ballistic trajectory, vertical case).
            % Zero PWM -> zero thrust -> pure gravity. NED (+z down): the
            % down-coordinate follows z(t) = z0 + 1/2 g t^2 (altitude falls),
            % and attitude must stay put. Catches a gravity sign flip,
            % quaternion drift, and any integrator miswiring.
            p    = params();
            alt0 = 10; Tf = 1.0; dt = 1e-3;   % start 10 m up -> z0 = -alt0 (NED)
            x0   = struct('r', [0 0 -alt0], 'v', [0 0 0], ...
                          'q', [1 0 0 0], 'omega', [0 0 0]);

            out = run_plant_case(@(t) [0 0 0 0], x0, Tf, dt);
            s   = unpack_state(out.yout{1}.Values.Data);

            z_expect = -alt0 + 0.5 * p.g * Tf^2;
            tc.log(sprintf('freefall: z(end)=%.4f m (expect %.4f), |q-I|=%.1e', ...
                s.r(end, 3), z_expect, norm(s.q(end, :) - [1 0 0 0])));

            tc.verifyEqual(s.r(end, 3), z_expect, 'AbsTol', 1e-3, ...
                sprintf('z %.4f m should track free-fall %.4f m (NED, +z down)', ...
                        s.r(end, 3), z_expect));
            tc.verifyEqual(s.q(end, :), [1 0 0 0], 'AbsTol', 1e-6, ...
                'Attitude should not drift with zero torque');
        end

        function ballisticTrajectory(tc)
            % Verifies REQ-PLANT-001 (ballistic trajectory, angled case).
            % Zero PWM -> zero thrust -> pure gravity. NED (+z down): the
            % down-coordinate follows z(t) = z0 + 1/2 g t^2 (altitude falls),
            % and the horizontal coordinates follow x(t) = x0 + vx0 t,
            % y(t) = y0 + vy0 t. Catches a gravity sign flip, quaternion drift,
            % and any integrator miswiring.
            p    = params();
            alt0 = 10; Tf = 1.0; dt = 1e-3;   % start 10 m up -> z0 = -alt0 (NED)
            vx0  = 1.5; vy0 = -2.5;
            x0   = struct('r', [0 0 -alt0], 'v', [vx0 vy0 0], ...
                          'q', [1 0 0 0], 'omega', [0 0 0]);

            out = run_plant_case(@(t) [0 0 0 0], x0, Tf, dt);
            s   = unpack_state(out.yout{1}.Values.Data);

            z_expect = -alt0 + 0.5 * p.g * Tf^2;
            x_expect = vx0 * Tf;
            y_expect = vy0 * Tf;
            tc.log(sprintf(['ballistic: r(end)=[%.4f %.4f %.4f] m ' ...
                '(expect [%.4f %.4f %.4f]), |q-I|=%.1e'], ...
                s.r(end, 1), s.r(end, 2), s.r(end, 3), ...
                x_expect, y_expect, z_expect, ...
                norm(s.q(end, :) - [1 0 0 0])));

            tc.verifyEqual(s.r(end, :), [x_expect y_expect z_expect], ...
                'AbsTol', 1e-3, ...
                sprintf('r [%+.4f %+.4f %+.4f] m should track ballistic [%+.4f %+.4f %+.4f] m (NED)', ...
                        s.r(end, :), [x_expect y_expect z_expect]));
            tc.verifyEqual(s.q(end, :), [1 0 0 0], 'AbsTol', 1e-6, ...
                'Attitude should not drift with zero torque');
        end

        %% REQ-PLANT-002: mechanical energy conservation (zero thrust, zero initial rates)
        
        function zeroTorqueAttitude(tc)
            % Verifies REQ-PLANT-002 (mechanical energy conservation).
            % Zero thrust, no drag -> gravity is the only force, and gravity
            % is CONSERVATIVE, so E = KE + PE is constant. NED (+z down):
            % altitude h = -z, so PE = m*g*h = -m*g*z. Datum is arbitrary and
            % there is no ground in the model, so the vehicle falls freely and
            % E holds for the whole run.
            p    = params();
            x0 = struct('r', [0 0 -10], 'v', [0 0 0], ...
                        'q', [1 0 0 0], 'omega', [0 0 0]);
            Tf = 1.0; dt = 1e-3;

            out = run_plant_case(@(t) [0 0 0 0], x0, Tf, dt);
            s   = unpack_state(out.yout{1}.Values.Data);

            % Mechanical energy calculation
            KE = 0.5 * p.m * vecnorm(s.v, 2, 2).^2;    % kinetic energy
            PE = - p.m * p.g * s.r(:, 3);              % potential energy
            ME = KE + PE;                              % mechanical energy

            tc.log(sprintf('mechanical energy: |E|(0)=%.1e, |E|(end)=%.1e, max rel drift=%.2e', ...
                ME(1), ME(end), max(abs(ME - ME(1)) / ME(1))));

            tc.verifyEqual(ME(end), ME(1), 'AbsTol', 1e-3, ...
                'Mechanical energy should be conserved with zero torque');
        end


        %% REQ-PLANT-003: angular momentum conservation (gyroscopic precession)

        function angularMomentumConservation(tc)
            % Verifies REQ-PLANT-003 (angular momentum conservation).
            % Zero PWM -> zero torque. For an asymmetric body (Ixx=Iyy /= Izz)
            % it is the ANGULAR MOMENTUM that is conserved:
            % an off-principal-axis spin precesses per Euler's equations (only
            % omega_z is constant here; omega_x,omega_y rotate about body z at
            % omega_z*(Izz-Ixx)/Ixx). We check the invariant that actually
            % holds -- |I*omega| constant -- which also exercises the
            % cross(omega, I*omega) gyroscopic term. (Body-rate constancy is
            % REQ-PLANT-004, and needs a single-principal-axis initial spin.)
            p  = params();
            x0 = struct('r', [0 0 0], 'v', [0 0 0], ...
                        'q', [1 0 0 0], 'omega', [1 -2 3]);

            Tf = 1.0; dt = 1e-3;
            out = run_plant_case(@(t) [0 0 0 0], x0, Tf, dt);
            s   = unpack_state(out.yout{1}.Values.Data);

            % Body-frame angular momentum L = I*omega (I diagonal & symmetric,
            % so row-wise this is omega*I). |L_world| = |L_body| because
            % rotation preserves norm, so this magnitude check is
            % frame-independent and needs no quaternion.
            Lmag = vecnorm(s.omega * p.I, 2, 2);   % |I*omega| per timestep
            rel  = abs(Lmag - Lmag(1)) / Lmag(1);
            tc.log(sprintf(['angular-momentum: |L|(0)=%.5f, |L|(end)=%.5f, ' ...
                'max rel drift=%.2e; omega(end)=[%+.3f %+.3f %+.3f] (precessed)'], ...
                Lmag(1), Lmag(end), max(rel), ...
                s.omega(end, 1), s.omega(end, 2), s.omega(end, 3)));

            tc.verifyLessThan(max(rel), 1e-3, sprintf( ...
                '|I*omega| drifted %.2e (rel) over %.1f s -- angular momentum not conserved', ...
                max(rel), Tf));
        end

        %% REQ-PLANT-004: Torque-free body rates (single-principal-axis spin)

        function singleAxisSpin(tc)
            % Verifies REQ-PLANT-004 (torque-free body rates).
            % Zero PWM -> zero torque. For a single-principal-axis spin
            % (omega along a principal axis) the body rates must remain
            % constant. This is the special case of REQ-PLANT-003 where the
            % precession term is zero and omega is constant.
            p  = params();
            x0 = struct('r', [0 0 0], 'v', [0 0 0], ...
                        'q', [1 0 0 0], 'omega', [0 0 3]);   % spin about z

            Tf = 1.0; dt = 1e-3;
            out = run_plant_case(@(t) [0 0 0 0], x0, Tf, dt);
            s   = unpack_state(out.yout{1}.Values.Data);

            rel = abs(s.omega(end, :) - s.omega(1, :)) ./ abs(s.omega(1, :));
            tc.log(sprintf(['single-axis spin: omega(end)=[%+.3f %+.3f %+.3f] rad/s, ' ...
                'max rel drift=%.2e'], ...
                s.omega(end, 1), s.omega(end, 2), s.omega(end, 3), max(rel)));

            tc.verifyLessThan(max(rel), 1e-3, sprintf( ...
                'Body rates drifted %.2e (rel) over %.1f s -- torque-free spin not constant', ...
                max(rel), Tf));
        end

        %% REQ-PLANT-005: Gyroscopic Coupling

        function gyroscopicCoupling(tc)
            % Verifies REQ-PLANT-005 (gyroscopic coupling).
            % Spin about body z at omega_z and apply a constant pure roll
            % moment M_x (from the two right-side motors). Linearized Euler
            % (Ix=Iy) gives omega_y(t) = omega_y_ss*(1 - cos(omega_z t)) with
            % omega_y_ss = M_x/((Iz-Iy)*omega_z), so omega_y keeps the sign of
            % omega_y_ss and its mean over whole nutation periods equals
            % omega_y_ss; omega_x nutates about zero. Exercises the
            % cross(omega, I*omega) gyroscopic term.
            p  = params();
            Iy = p.I(2, 2); Iz = p.I(3, 3);
            omega_z = 3;   % spin about z
            u = [0.001 0 0 0.001];
            T = pwm_to_thrust(u, p);
            A = motor_mixing_matrix(p);
            M = A(2:4, :) * T';  % net torque from the commanded thrust vector
            Mx = M(1);
            omega_y_ss = Mx / ((Iz - Iy) * omega_z);  % steady-state omega_y from Euler's equations



            period = 2*pi/omega_z;  % period of precession about z
            Tf = 5*period; dt = 1e-3;


            x0 = struct('r', [0 0 0], 'v', [0 0 0], ...
                        'q', [1 0 0 0], 'omega', [0 0 omega_z]);   % spin about z

            out = run_plant_case(@(t) u, x0, Tf, dt);
            t = out.yout{1}.Values.Time;
            s   = unpack_state(out.yout{1}.Values.Data);

            warm = t > period;  % ignore the first precession period, which is transient
            omega_y_mean = mean(s.omega(warm, 2));
            rel = abs(omega_y_mean - omega_y_ss) / abs(omega_y_ss);
            tc.log(sprintf(['gyro: M=[%+.4f %+.4f %+.4f] N*m, omega_y_ss=[%+.3f] rad/s, ' ...
                'mean omega_y=[%+.3f] rad/s, max |omega_x|=%.3e'], ...
                M(1), M(2), M(3), omega_y_ss, omega_y_mean, rel, max(abs(s.omega(warm,1)))));

            % Sign: precession is in the predicted direction
            tc.verifyGreaterThan(omega_y_mean * omega_y_ss, 0, ...
                'omega_y precession is in the wrong direction');
            
            % Magnitude: precession is within 10% of the predicted value
            tc.verifyLessThan(rel, 0.1, ...
                'mean omega_y not within 10%% of M_x/((Iz-Iy)*omega_z) prediction');

        end

        %% REQ-PLANT-006: Quaternion norm integrity

        function quaternionNormCheck(tc)
            % Verifies REQ-PLANT-006 (quaternion norm integrity).
            % The quaternion must remain unit-length. This is a check on the
            % quaternion integrator and the quat_derivative function.
            p  = params();
            x0 = struct('r', [0 0 0], 'v', [0 0 0], ...
                        'q', [1 0 0 0], 'omega', [1 -2 3]);

            Tf = 1.0; dt = 1e-3;
            out = run_plant_case(@(t) [0 0 0 0], x0, Tf, dt);
            s   = unpack_state(out.yout{1}.Values.Data);

            qnorm = vecnorm(s.q, 2, 2);
            rel   = abs(qnorm - 1);
            tc.log(sprintf('quaternion norm: |q|(end)=%.6f, max rel drift=%.2e', ...
                qnorm(end), max(rel)));

            tc.verifyLessThan(max(rel), 1e-6, sprintf( ...
                'Quaternion norm drifted %.2e (rel) over %.1f s -- quaternion not unit-length', ...
                max(rel), Tf));
        end
    end
end

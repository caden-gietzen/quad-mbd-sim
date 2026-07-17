function out = run_plant_case(u_fn, x0, Tf, dt)
    % Drive plant_test.slx open-loop with a scripted PWM input and initial
    % state, keeping the model a pure, source-free component: the stimulus
    % and the starting state are injected here via Simulink.SimulationInput,
    % so each scenario is data rather than a model edit.
    %
    %   u_fn : function handle @(t) -> 1x4 normalized PWM command (0..1),
    %          one column per motor.
    %   x0   : struct with fields r, v, q, omega giving the initial state
    %          (r, v, omega each 1x3; q 1x4 scalar-first). These are wired
    %          to the plant integrator initial conditions through the
    %          base-workspace variables x0_r / x0_v / x0_q / x0_omega -- the
    %          integrator "Initial condition" fields in plant_test.slx must
    %          reference exactly those variable names for this to take effect.
    %   Tf   : stop time (s).
    %   dt   : sample spacing (s) for the External Input time vector.
    %
    % Returns the raw Simulink.SimulationOutput. Feed
    %   out.yout{1}.Values.Data
    % (the logged 13-state, N-by-13) through unpack_state to get named
    % histories. Requires Output logging (yout) enabled in the model's
    % Data Import/Export configuration.
    arguments
        u_fn (1, 1) function_handle
        x0   (1, 1) struct
        Tf   (1, 1) double
        dt   (1, 1) double
    end

    % Build the External Input as a Dataset with one vector-valued element.
    % The [t u] matrix form can't drive a width-4 Inport -- it maps each
    % data column to a separate scalar port and errors on a vector port. A
    % Dataset element carrying an N-by-4 timeseries maps cleanly onto the
    % single width-4 root Inport. The element name must match the Inport
    % block name ('u' here); rename it if the port is named differently.
    t = (0:dt:Tf)';
    u = zeros(numel(t), 4);
    for k = 1:numel(t)
        u(k, :) = u_fn(t(k));
    end

    ds = Simulink.SimulationData.Dataset;
    ds = ds.addElement(timeseries(u, t), 'u');

    simIn = Simulink.SimulationInput('plant_test');
    simIn = simIn.setExternalInput(ds);
    simIn = simIn.setModelParameter('StopTime', num2str(Tf));
    simIn = simIn.setVariable('x0_r',     x0.r);
    simIn = simIn.setVariable('x0_v',     x0.v);
    simIn = simIn.setVariable('x0_q',     x0.q);
    simIn = simIn.setVariable('x0_omega', x0.omega);

    out = sim(simIn);
end

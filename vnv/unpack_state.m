function s = unpack_state(X)
    % Single source of truth for plant_test.slx's 13-element state-vector
    % layout. Column order matches plant()'s argument/derivative ordering:
    %
    %    1:3   r      position     [x y z], NED (+z down)  (m)
    %    4:6   v      velocity                      (m/s)
    %    7:10  q      attitude     scalar-first [w x y z]
    %   11:13  omega  body rates                    (rad/s)
    %
    % X is N-by-13 (a logged time history) or 1-by-13 (a single sample).
    % Every test indexes state through here, not with raw column numbers,
    % so a future reordering of the Mux feeding the state Outport is a
    % one-line change instead of a silent break across the whole suite.
    %
    % NOTE: this ordering must match the concatenation order of the Mux (or
    % bus) wired into plant_test.slx's state Outport. Confirm once in the
    % model; the tests assume it.
    %
    % Accepts whatever shape Simulink logs the state as and normalizes to
    % N-by-13: a 2-D (1x13) state signal logs as 1x13xN, a 1-D width-13
    % signal logs as Nx13 -- both collapse to Nx13 here.
    arguments
        X double
    end
    X = squeeze(X);                             % 1x13xN -> 13xN; Nx13 unchanged
    if size(X, 1) == 13 && size(X, 2) ~= 13
        X = X.';                                % 13xN -> Nx13
    end
    assert(ismatrix(X) && size(X, 2) == 13, 'unpack_state:BadShape', ...
        'Expected a 13-wide state; got size [%s].', num2str(size(X)));

    s.r     = X(:, 1:3);
    s.v     = X(:, 4:6);
    s.q     = X(:, 7:10);
    s.omega = X(:, 11:13);
end

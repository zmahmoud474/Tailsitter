function [alpha_trim, delta_trim] = trim_block(L, M, W, alpha0, delta0)
% TRIM_BLOCK computes trim alpha and delta from L and M with initial guesses
% Inputs:
%   L       : lift [N]
%   M       : pitching moment [Nm]
%   W       : weight [N]
%   alpha0  : initial guess for alpha [rad]
%   delta0  : initial guess for control [rad]
%
% Outputs:
%   alpha_trim : trim angle of attack [rad]
%   delta_trim : trim control surface deflection [rad]

% ==== Gains for iterative solver ====
k_alpha = 0.01;  % alpha loop
k_delta = 0.001; % delta loop

% ==== Use input guesses ====
alpha = alpha0;
delta = delta0;

% ==== Iterative solver ====
for i = 1:100
    % Errors
    Fz = L - W;   % vertical force error
    % Moment error is just M

    % Convergence check
    if abs(Fz) < 1e-3 && abs(M) < 1e-4
        break;
    end

    % Update guesses
    alpha = alpha - k_alpha * M;
    delta = delta - k_delta * Fz;

    % Optional saturation
    alpha = max(min(alpha, deg2rad(10)), deg2rad(-10));
    delta = max(min(delta, deg2rad(20)), deg2rad(-20));
end

% Output
alpha_trim = alpha;
delta_trim = delta;

end
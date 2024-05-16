
function dy = getForceDerivativesSym()
    % Define symbolic variables
    syms vx vy r delta lf Df Cf Bf real

    % Compute alpha_f
    alpha_f = -atan2(vy + r * lf, vx) + delta;

    % Expression for F_y
    F_y = Df * sin(Cf * atan(Bf * alpha_f));

    % Compute derivatives
    dFy_dvx = diff(F_y, vx);
    dFy_dvy = diff(F_y, vy);
    dFy_dr = diff(F_y, r);

    % Return derivatives as output
    dy = [dFy_dvx; dFy_dvy; dFy_dr];
end

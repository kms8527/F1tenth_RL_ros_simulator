% Define vehicle parameters
car.l_f = 1.5;   % distance from front axle to CG
car.l_r = 1.5;   % distance from rear axle to CG
car.I_z = 1500;  % vehicle's moment of inertia about the z-axis

% Define fixed state values
fixed_vel = 10;
fixed_yawdot = 0.1;
fixed_slip_angle = 0.05;
fixed_steer = 0.1;

% Define variable range for plotting
variable_range = linspace(-0.2, 0.2, 100);

% Initialize arrays to store results
results_nonlinear = zeros(4, length(variable_range));
results_linear = zeros(4, length(variable_range));

% Compute nonlinear and linear responses
for i = 1:length(variable_range)
    % Scenario 1: Change in velocity
    results_nonlinear(1, i) = compute_yawdot(fixed_vel + variable_range(i), fixed_yawdot, fixed_slip_angle, fixed_steer, car);
    results_linear(1, i) = compute_linear_yawdot(fixed_vel, fixed_yawdot, fixed_slip_angle, fixed_steer, variable_range(i), 0, 0, 0, car);

    % Scenario 2: Change in yaw rate
    results_nonlinear(2, i) = compute_yawdot(fixed_vel, fixed_yawdot + variable_range(i), fixed_slip_angle, fixed_steer, car);
    results_linear(2, i) = compute_linear_yawdot(fixed_vel, fixed_yawdot, fixed_slip_angle, fixed_steer, 0, variable_range(i), 0, 0, car);

    % Scenario 3: Change in slip angle
    results_nonlinear(3, i) = compute_yawdot(fixed_vel, fixed_yawdot, fixed_slip_angle + variable_range(i), fixed_steer, car);
    results_linear(3, i) = compute_linear_yawdot(fixed_vel, fixed_yawdot, fixed_slip_angle, fixed_steer, 0, 0, variable_range(i), 0, car);

    % Scenario 4: Change in steering angle
    results_nonlinear(4, i) = compute_yawdot(fixed_vel, fixed_yawdot, fixed_slip_angle, fixed_steer + variable_range(i), car);
    results_linear(4, i) = compute_linear_yawdot(fixed_vel, fixed_yawdot, fixed_slip_angle, fixed_steer, 0, 0, 0, variable_range(i), car);
end

% Plot the results
figure;
for j = 1:4
    subplot(2, 2, j);
    plot(variable_range, results_nonlinear(j, :), 'b', variable_range, results_linear(j, :), 'r--');
    legend('Nonlinear', 'Linear');
    title(['Response with respect to changing ', {'velocity', 'yaw rate', 'slip angle', 'steering'}{j}]);
    xlabel('Variable change');
    ylabel('Yawdot output');
end

function yawdot = compute_yawdot(v, yawdot, slip_angle, steer, car)
    % Placeholder for actual yawdot computation
    yawdot = (car.l_f * v * cos(steer) - car.l_r * yawdot) / car.I_z;  % Simplified placeholder
end

function lin_yawdot = compute_linear_yawdot(v, yawdot, slip, steer, dv, dyawdot, dslip, dsteer, car)
    % Compute linearized yawdot based on small deviations from fixed states
    lin_yawdot = dfyawdot_dv * dv + dfyawdot_dyawdot * dyawdot + dfyawdot_dslip * dslip + dfyawdot_dsteer * dsteer ...
                 + dfyawdot_dv * (v - fixed_vel) + dfyawdot_dyawdot * (yawdot - fixed_yawdot) ...
                 + dfyawdot_dslip * (slip - fixed_slip) + dfyawdot_dsteer * fixed_steer;
end


% Plot the results
figure;
for j = 1:4
    subplot(2, 2, j);
    plot(variable_range, results_nonlinear(j, :), 'b', variable_range, results_linear(j, :), 'r--');
    legend('Nonlinear', 'Linear');
    title(['Response with respect to changing ', {'velocity', 'yaw rate', 'slip angle', 'steering'}{j}]);
    xlabel('Variable change');
    ylabel('Yawdot output');
end

function yawdot = compute_yawdot(v, yawdot, slip_angle, steer, car)
    % Real nonlinear function implementation needed here
    % Placeholder for actual yawdot computation
    yawdot = (car.l_f * v * cos(steer) - car.l_r * yawdot) / car.I_z;  % Simplified placeholder
end

function lin_yawdot = compute_linear_yawdot(df_dv, df_dyawdot, df_dslip, df_dsteer, v, yawdot, slip, steer, dv, dyawdot, dslip, dsteer)
    % Compute linearized yawdot based on small deviations from fixed states
    lin_yawdot = df_dv * dv + df_dyawdot * dyawdot + df_dslip * dslip + df_dsteer * dsteer ...
                 + df_dv * (v - fixed_vel) + df_dyawdot * (yawdot - fixed_yawdot) ...
                 + df_dslip * (slip - fixed_slip) + df_dsteer * fixed_steer;
end


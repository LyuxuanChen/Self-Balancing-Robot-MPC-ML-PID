%% Self-Balancing Robot Dataset Resampling Script

clear; clc;

%% STEP 1: Load Dataset

filename = 'robot_dataset_20Bestuptonow.csv';

data = readtable(filename);

% Extract original time vector
time = data.time;

% Remove duplicate timestamps while preserving order
[time, uniqIdx] = unique(time, 'stable');

data = data(uniqIdx,:);


%% STEP 2: Define Uniform Time Grid

dt = 0.01;   % desired uniform time step [s]

% Construct a numerically stable uniform time grid
N_uniform = floor((time(end) - time(1))/dt) + 1;

time_uniform = time(1) + (0:N_uniform-1)' * dt;


%% STEP 3: Resample All Relevant Variables

vars = { ...
    'sin_theta', ...
    'cos_theta', ...
    'theta_dot', ...
    'x', ...
    'x_dot', ...
    'u' ...
    };

data_resampled = table( ...
    time_uniform, ...
    'VariableNames', {'time'} ...
    );


for i = 1:length(vars)

    original = data.(vars{i});

    interpolated = interp1( ...
        time, ...
        original, ...
        time_uniform, ...
        'linear' ...
        );

    data_resampled.(vars{i}) = interpolated;

end


%% STEP 4: Re-normalize sin(theta) and cos(theta)

% Linear interpolation can move sin/cos slightly away from the unit circle.
% Re-normalize so that:
%
% sin(theta)^2 + cos(theta)^2 = 1

mag = sqrt( ...
    data_resampled.sin_theta.^2 + ...
    data_resampled.cos_theta.^2 ...
    ) + eps;

data_resampled.sin_theta = ...
    data_resampled.sin_theta ./ mag;

data_resampled.cos_theta = ...
    data_resampled.cos_theta ./ mag;


%% STEP 5: Recompute Next-Step Targets

N = height(data_resampled);

if N < 2
    error('Not enough samples after resampling to build next-step targets.');
end


% Next-step states correspond exactly to t + dt

sin_theta_next = ...
    data_resampled.sin_theta(2:N);

cos_theta_next = ...
    data_resampled.cos_theta(2:N);

theta_dot_next = ...
    data_resampled.theta_dot(2:N);

x_next = ...
    data_resampled.x(2:N);

x_dot_next = ...
    data_resampled.x_dot(2:N);


% Trim current signals so they align with the next-step targets

out = table();

out.time = ...
    data_resampled.time(1:N-1);

out.sin_theta = ...
    data_resampled.sin_theta(1:N-1);

out.cos_theta = ...
    data_resampled.cos_theta(1:N-1);

out.theta_dot = ...
    data_resampled.theta_dot(1:N-1);

out.x = ...
    data_resampled.x(1:N-1);

out.x_dot = ...
    data_resampled.x_dot(1:N-1);

out.u = ...
    data_resampled.u(1:N-1);


% Targets at t + dt

out.sin_theta_next = sin_theta_next;

out.cos_theta_next = cos_theta_next;

out.theta_dot_next = theta_dot_next;

out.x_next = x_next;

out.x_dot_next = x_dot_next;


%% STEP 6: Save Resampled Dataset

new_filename = [ ...
    'robot_dataset_resampled_', ...
    datestr(now, 'yyyymmdd_HHMMSS'), ...
    '.csv' ...
    ];

writetable(out, new_filename);

disp(['Resampled data saved to ', new_filename]);


%% STEP 7: Verify Uniform Time Step

dt_check = diff(out.time);

fprintf('\n=== Resampling Check ===\n');

fprintf('Target dt:  %.6f s\n', dt);
fprintf('Minimum dt: %.6f s\n', min(dt_check));
fprintf('Median dt:  %.6f s\n', median(dt_check));
fprintf('Maximum dt: %.6f s\n', max(dt_check));


%% STEP 8: Plot Original vs Resampled Signals

figure('Name','Original vs Resampled Signals', ...
       'Position',[100 100 1300 900]);


subplot(3,2,1);

plot(time, data.sin_theta, 'b-');
hold on;

plot(out.time, out.sin_theta, 'r--');

legend('Original','Resampled','Location','best');

title('sin(\theta)');

xlabel('Time (s)');
ylabel('sin(\theta)');

grid on;


subplot(3,2,2);

plot(time, data.cos_theta, 'b-');
hold on;

plot(out.time, out.cos_theta, 'r--');

legend('Original','Resampled','Location','best');

title('cos(\theta)');

xlabel('Time (s)');
ylabel('cos(\theta)');

grid on;


subplot(3,2,3);

plot(time, data.theta_dot, 'b-');
hold on;

plot(out.time, out.theta_dot, 'r--');

legend('Original','Resampled','Location','best');

title('Angular Velocity');

xlabel('Time (s)');
ylabel('\theta dot (rad/s)');

grid on;


subplot(3,2,4);

plot(time, data.x, 'b-');
hold on;

plot(out.time, out.x, 'r--');

legend('Original','Resampled','Location','best');

title('Position');

xlabel('Time (s)');
ylabel('x (m)');

grid on;


subplot(3,2,5);

plot(time, data.x_dot, 'b-');
hold on;

plot(out.time, out.x_dot, 'r--');

legend('Original','Resampled','Location','best');

title('Linear Velocity');

xlabel('Time (s)');
ylabel('x dot (m/s)');

grid on;


subplot(3,2,6);

plot(time, data.u, 'b-');
hold on;

plot(out.time, out.u, 'r--');

legend('Original','Resampled','Location','best');

title('Control Input');

xlabel('Time (s)');
ylabel('u');

grid on;


sgtitle('Original vs Uniformly Resampled Dataset');


%% STEP 9: Compare Reconstructed Theta

% Original wrapped angle reconstructed from sin/cos

theta_gt_wrapped = atan2( ...
    data.sin_theta, ...
    data.cos_theta ...
    );


% Unwrap BEFORE interpolation to avoid problems near +/- pi

theta_gt_unwrapped = unwrap(theta_gt_wrapped);


% Interpolate original continuous theta onto the uniform grid

theta_gt_unwrapped_u = interp1( ...
    time, ...
    theta_gt_unwrapped, ...
    out.time, ...
    'linear' ...
    );


% Reconstruct theta from the resampled sin/cos

theta_resampled_wrapped = atan2( ...
    out.sin_theta, ...
    out.cos_theta ...
    );

theta_resampled_unwrapped = ...
    unwrap(theta_resampled_wrapped);


%% Plot theta comparison

figure('Name','Theta Verification');


subplot(2,1,1);

plot( ...
    out.time, ...
    wrapToPi(theta_gt_unwrapped_u), ...
    'b-', ...
    'LineWidth',1.1 ...
    );

hold on;

plot( ...
    out.time, ...
    theta_resampled_wrapped, ...
    'r--', ...
    'LineWidth',1.1 ...
    );

legend( ...
    '\theta_{original} wrapped', ...
    '\theta_{resampled} wrapped', ...
    'Location','best' ...
    );

xlabel('Time (s)');
ylabel('\theta (rad)');

title('Wrapped Angle');

grid on;


subplot(2,1,2);

plot( ...
    out.time, ...
    theta_gt_unwrapped_u, ...
    'b-', ...
    'LineWidth',1.1 ...
    );

hold on;

plot( ...
    out.time, ...
    theta_resampled_unwrapped, ...
    'r--', ...
    'LineWidth',1.1 ...
    );

legend( ...
    '\theta_{original} unwrapped', ...
    '\theta_{resampled} unwrapped', ...
    'Location','best' ...
    );

xlabel('Time (s)');
ylabel('\theta (rad)');

title('Unwrapped Angle');

grid on;


%% STEP 10: Verify Unit-Circle Constraint

circle_error = ...
    abs(out.sin_theta.^2 + out.cos_theta.^2 - 1);

fprintf('\n=== sin/cos Unit Circle Check ===\n');

fprintf('Maximum error: %.8e\n', max(circle_error));
fprintf('Mean error:    %.8e\n', mean(circle_error));


%% STEP 11: Final Summary

fprintf('\n=== Dataset Summary ===\n');

fprintf('Original samples:   %d\n', height(data));
fprintf('Resampled samples:  %d\n', height(out));

fprintf('Start time: %.4f s\n', out.time(1));
fprintf('End time:   %.4f s\n', out.time(end));

fprintf('Uniform timestep: %.4f s\n', dt);

fprintf('\n Dataset is ready for LSTM training.\n');

%% ============================================================
% Self-Balancing Robot - Pure Recovery Data Collection
%
% Model:
%   selfBalanceRobotRecovery
%
% Revolute Joint setup:
%   State Targets -> Specify Position Target = ON
%   Value = initial_angle_deg
%   Unit  = deg
%   Priority = High (desired)
%
% The script automatically changes initial_angle_deg
% before each simulation.
%
% Output CSV columns:
%   time
%   theta_deg
%   sin_theta
%   cos_theta
%   theta_dot
%   x
%   x_dot
%   u
%% ============================================================

clear;
clc;
close all;


%% ===================== Configuration =====================

model_name = 'selfBalanceRobotRecovery';

% Initial recovery angles to test
initial_angles_deg = [
     % 5
     % 10
     % 15
     % 20
     % 30
     % 40
     % 50
     % 60
     55
];


%% ===================== Load Simulink Model =====================

load_system(model_name);

fprintf('\n');
fprintf('========================================\n');
fprintf(' Pure Recovery Data Collection\n');
fprintf(' Model: %s\n', model_name);
fprintf('========================================\n');


%% ===================== Run All Recovery Tests =====================

for run_idx = 1:length(initial_angles_deg)

    %% --------------------------------------------------------
    % Set Initial Angle
    %% --------------------------------------------------------

    initial_angle_deg = initial_angles_deg(run_idx);

    fprintf('\n');
    fprintf('----------------------------------------\n');
    fprintf('Run %d of %d\n', ...
        run_idx, length(initial_angles_deg));

    fprintf('Requested initial angle: %+g deg\n', ...
        initial_angle_deg);

    fprintf('----------------------------------------\n');


    %% --------------------------------------------------------
    % Run Simulink Model
    %% --------------------------------------------------------

    fprintf('▶️ Running simulation...\n');

    sim(model_name);

    fprintf('✅ Simulation complete.\n');


    %% --------------------------------------------------------
    % Extract Logged Signals
    %% --------------------------------------------------------

    fprintf('📤 Extracting signals...\n');


    % Shared time vector
    time = AngleData.time;


    % Angle [rad]
    theta = AngleData.signals.values;


    % Angular velocity [rad/s]
    theta_dot = AngularVelocity.signals.values;


    % Cart position [m]
    x = DistanceData.signals.values;


    % Cart linear velocity [m/s]
    x_dot = LinearV.signals.values;


    % PID control components
    u_angle = ControlAngle.signals.values;

    u_dist = ControlDistance.signals.values;


    % Combined control input
    %
    % Same definition used in your previous extraction:
    %
    % u = PID_Angle - PID_Distance

    u = u_angle - u_dist;


    %% --------------------------------------------------------
    % Convert Everything to Column Vectors
    %% --------------------------------------------------------

    time = time(:);

    theta = theta(:);

    theta_dot = theta_dot(:);

    x = x(:);

    x_dot = x_dot(:);

    u_angle = u_angle(:);

    u_dist = u_dist(:);

    u = u(:);


    %% --------------------------------------------------------
    % Convert Angle Representation
    %% --------------------------------------------------------

    % Human-readable angle [deg]
    theta_deg = rad2deg(theta);


    % ML representation
    sin_theta = sin(theta);

    cos_theta = cos(theta);


    %% --------------------------------------------------------
    % Check Signal Lengths
    %% --------------------------------------------------------

    signal_lengths = [
        length(time)
        length(theta_deg)
        length(sin_theta)
        length(cos_theta)
        length(theta_dot)
        length(x)
        length(x_dot)
        length(u)
    ];

    if any(signal_lengths ~= signal_lengths(1))

        error('Logged signals do not all have the same number of samples.');

    end


    %% --------------------------------------------------------
    % Verify Initial Angle
    %% --------------------------------------------------------

    measured_initial_angle_deg = theta_deg(1);


    fprintf('\n=== Initial Angle Check ===\n');

    fprintf('Requested: %+8.3f deg\n', ...
        initial_angle_deg);

    fprintf('Measured:  %+8.3f deg\n', ...
        measured_initial_angle_deg);

    fprintf('Difference: %+8.5f deg\n', ...
        measured_initial_angle_deg - initial_angle_deg);


    %% --------------------------------------------------------
    % Display State Coverage
    %% --------------------------------------------------------

    fprintf('\n=== State Coverage ===\n');


    fprintf('Theta:\n');

    fprintf('  Minimum:       %+8.3f deg\n', ...
        min(theta_deg));

    fprintf('  Maximum:       %+8.3f deg\n', ...
        max(theta_deg));

    fprintf('  Maximum |θ|:    %8.3f deg\n', ...
        max(abs(theta_deg)));


    fprintf('\nTheta dot:\n');

    fprintf('  Minimum:       %+8.4f rad/s\n', ...
        min(theta_dot));

    fprintf('  Maximum:       %+8.4f rad/s\n', ...
        max(theta_dot));


    fprintf('\nControl input:\n');

    fprintf('  Minimum:       %+8.4f\n', ...
        min(u));

    fprintf('  Maximum:       %+8.4f\n', ...
        max(u));


    %% --------------------------------------------------------
    % Verify sin/cos Unit Circle
    %% --------------------------------------------------------

    circle_error = abs( ...
        sin_theta.^2 + ...
        cos_theta.^2 - 1 ...
        );


    fprintf('\n=== sin/cos Check ===\n');

    fprintf('Maximum unit-circle error: %.8e\n', ...
        max(circle_error));


    %% --------------------------------------------------------
    % Build RAW Dataset Table
    %% --------------------------------------------------------

    T = table( ...
        time, ...
        theta_deg, ...
        sin_theta, ...
        cos_theta, ...
        theta_dot, ...
        x, ...
        x_dot, ...
        u, ...
        'VariableNames', { ...
        'time', ...
        'theta_deg', ...
        'sin_theta', ...
        'cos_theta', ...
        'theta_dot', ...
        'x', ...
        'x_dot', ...
        'u'} ...
        );


    %% --------------------------------------------------------
    % Create Descriptive Filename
    %% --------------------------------------------------------

    if initial_angle_deg > 0

        angle_label = sprintf( ...
            'pos_%02ddeg', ...
            round(abs(initial_angle_deg)) ...
            );

    elseif initial_angle_deg < 0

        angle_label = sprintf( ...
            'neg_%02ddeg', ...
            round(abs(initial_angle_deg)) ...
            );

    else

        angle_label = '0deg';

    end


    timestamp = datestr( ...
        now, ...
        'yyyymmdd_HHMMSS' ...
        );


    filename = sprintf( ...
        'recovery_%s_raw_%s.csv', ...
        angle_label, ...
        timestamp ...
        );


    %% --------------------------------------------------------
    % Save CSV
    %% --------------------------------------------------------

    writetable(T, filename);


    fprintf('\n💾 Saved dataset:\n');

    fprintf('   %s\n', filename);


    %% --------------------------------------------------------
    % Quick Recovery Visualization
    %% --------------------------------------------------------

    figure( ...
        'Name', ...
        sprintf('Pure Recovery %+g deg', initial_angle_deg), ...
        'Position', ...
        [150 150 1000 800] ...
        );


    % ----- Angle -----

    subplot(3,1,1);

    plot( ...
        time, ...
        theta_deg, ...
        'LineWidth', ...
        1.2 ...
        );

    hold on;

    yline( ...
        0, ...
        'k--', ...
        'Target' ...
        );

    yline( ...
        2, ...
        'g:', ...
        '+2 deg' ...
        );

    yline( ...
        -2, ...
        'g:', ...
        '-2 deg' ...
        );

    xlabel('Time (s)');

    ylabel('\theta (deg)');

    title( ...
        sprintf( ...
        'Recovery from %+g°', ...
        initial_angle_deg ...
        ) ...
        );

    grid on;


    % ----- Angular velocity -----

    subplot(3,1,2);

    plot( ...
        time, ...
        theta_dot, ...
        'LineWidth', ...
        1.2 ...
        );

    hold on;

    yline( ...
        deg2rad(5), ...
        'g:', ...
        '+5 deg/s' ...
        );

    yline( ...
        -deg2rad(5), ...
        'g:', ...
        '-5 deg/s' ...
        );

    xlabel('Time (s)');

    ylabel('\theta dot (rad/s)');

    title('Angular Velocity');

    grid on;


    % ----- Control -----

    subplot(3,1,3);

    plot( ...
        time, ...
        u, ...
        'LineWidth', ...
        1.2 ...
        );

    xlabel('Time (s)');

    ylabel('Control u');

    title('PID Control Input');

    grid on;


    sgtitle( ...
        sprintf( ...
        'Pure Recovery Dataset: %+g° Initial Angle', ...
        initial_angle_deg ...
        ) ...
        );


    %% --------------------------------------------------------
    % Recovery Settling-Time Check
    %% --------------------------------------------------------

    % Our agreed comparison criterion:
    %
    % |theta| <= 2 deg
    % |theta_dot| <= 5 deg/s
    % continuously for at least 1 second


    angle_tolerance_deg = 2;

    velocity_tolerance_rad_s = deg2rad(5);

    required_stable_time = 1.0;


    stable_sample = ...
        abs(theta_deg) <= angle_tolerance_deg & ...
        abs(theta_dot) <= velocity_tolerance_rad_s;


    % Find continuous stable regions

    transitions = diff( ...
        [false; stable_sample; false] ...
        );


    region_starts = find( ...
        transitions == 1 ...
        );


    region_ends = find( ...
        transitions == -1 ...
        ) - 1;


    settle_time = NaN;


    for region_idx = 1:length(region_starts)

        start_idx = region_starts(region_idx);

        end_idx = region_ends(region_idx);


        stable_duration = ...
            time(end_idx) - time(start_idx);


        if stable_duration >= required_stable_time

            settle_time = time(start_idx);

            break;

        end

    end


    fprintf('\n=== Recovery Performance ===\n');


    if isnan(settle_time)

        fprintf( ...
            'Settling criterion NOT reached.\n' ...
            );

    else

        fprintf( ...
            'Settling time: %.4f s\n', ...
            settle_time ...
            );

    end


    fprintf( ...
        'Final angle: %.4f deg\n', ...
        theta_deg(end) ...
        );


    fprintf( ...
        'Final angular velocity: %.6f rad/s\n', ...
        theta_dot(end) ...
        );


    fprintf( ...
        'RMS angle: %.4f deg\n', ...
        sqrt(mean(theta_deg.^2)) ...
        );


    fprintf( ...
        'Peak |u|: %.4f\n', ...
        max(abs(u)) ...
        );


    fprintf( ...
        'RMS u: %.4f\n', ...
        sqrt(mean(u.^2)) ...
        );


    fprintf('\n✅ Run complete.\n');

end


%% ===================== Finished =====================

fprintf('\n');
fprintf('========================================\n');
fprintf('✅ All pure recovery runs completed.\n');
fprintf('========================================\n');

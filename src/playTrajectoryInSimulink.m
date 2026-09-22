%% Self-Balancing Robot Data Extraction Script

% === STEP 1: Run the Simulink Model ===
disp('Running simulation');
sim('selfBalanceRobotverylargedisturbance');  % Adjust model name if needed

% === STEP 2: Extract Data from Workspace ===
disp('Extracting signals...');

% Shared time vector
time = AngleData.time;

% Extract logged signals
theta      = AngleData.signals.values;
theta_dot  = AngularVelocity.signals.values;
x          = DistanceData.signals.values;
x_dot      = LinearV.signals.values;

% New: Extract angle and distance control components
u_angle    = ControlAngle.signals.values;
u_dist     = ControlDistance.signals.values;

% Combine using formula: u = PID_Angle - PID_Distance
u = u_angle - u_dist;

% === STEP 3: Convert theta -> sin(theta), cos(theta)
sin_theta     = sin(theta);
cos_theta     = cos(theta);
sin_theta_n   = sin(theta(2:end));
cos_theta_n   = cos(theta(2:end));

% === STEP 4: Build ML Dataset
X = [sin_theta(1:end-1), cos_theta(1:end-1), theta_dot(1:end-1), ...
     x(1:end-1), x_dot(1:end-1), u(1:end-1)];
Y = [sin_theta_n, cos_theta_n, theta_dot(2:end), ...
     x(2:end), x_dot(2:end)];

% === STEP 5: Save as CSV
disp('Saving dataset...');

T = array2table([time(1:end-1), X, Y], ...
    'VariableNames', {'time', ...
                      'sin_theta', 'cos_theta', 'theta_dot', 'x', 'x_dot', 'u', ...
                      'sin_theta_next', 'cos_theta_next', 'theta_dot_next', 'x_next', 'x_dot_next'});

% Filename with timestamp
csv_filename = ['robot_dataset_', datestr(now, 'yyyymmdd_HHMMSS'), '.csv'];
writetable(T, csv_filename);
disp(['Saved to ', csv_filename]);

%% === STEP 6: Verify Angular Velocity ===

disp('Checking angular velocity signal...');

% Independent numerical derivative from logged angle
theta_unwrapped = unwrap(theta);
theta_dot_check = gradient(theta_unwrapped, time);

% Plot logged angular velocity vs numerical derivative
figure('Name','Angular Velocity Verification');

plot(time, theta_dot, 'LineWidth', 1.2);
hold on;

plot(time, theta_dot_check, '--', 'LineWidth', 1.2);

legend(...
    'Logged AngularVelocity', ...
    'Numerical gradient(\theta)', ...
    'Location','best');

xlabel('Time (s)');
ylabel('\theta dot (rad/s)');

title('Angular Velocity Verification');

grid on;

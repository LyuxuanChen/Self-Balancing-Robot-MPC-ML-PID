%% PlaybackTrajectory.m

angle_deg = 5;

trajectoryFile = sprintf( ...
    'Angle_%ddeg_trajectory.csv', ...
    angle_deg);

%% Read trajectory

if exist(trajectoryFile, 'file') ~= 2
    error('Cannot find trajectory file: %s', ...
        trajectoryFile);
end

T = readtable(trajectoryFile);

theta_sim = timeseries( ...
    deg2rad(T.Theta_deg), ...
    T.Time);

% Explicitly make it available to Simulink
assignin('base', 'theta_sim', theta_sim);

%% Use the currently opened Simulink model

modelName = ...
    'selfBalanceRobot_DemonstrationFromData';

if ~bdIsLoaded(modelName)
    load_system(modelName);
end

open_system(modelName);

%% Set stop time to a numerical value

stopTime = num2str(T.Time(end), '%.15g');

set_param(modelName, ...
    'StopTime', stopTime);

%% Run model

fprintf('Loaded %s\n', trajectoryFile);
fprintf('Running playback for %s seconds...\n', stopTime);

simOut = sim(modelName, ...
    'SrcWorkspace', 'base');

fprintf('Playback completed.\n');
%% LSTM Training for Self-Balancing Robot Dynamics
% Inputs:
%   [sin(theta), cos(theta), theta_dot, u]_t
%
% Targets:
%   [sin(theta), cos(theta), theta_dot]_(t+1)
%
% Dataset timestep:
%   dt = 0.01 s

clear;
clc;


%% STEP 1: Load Dataset

filename = 'robot_dataset_resampled_20260907_231909';

data = readtable(filename);

time = data.time;

sin_theta = data.sin_theta;
cos_theta = data.cos_theta;
theta_dot = data.theta_dot;
u = data.u;

% Next-step targets
sin_theta_n = data.sin_theta_next;
cos_theta_n = data.cos_theta_next;
theta_dot_n = data.theta_dot_next;


%% STEP 2: Build Input and Target Matrices

% Inputs at time t
%
% [sin(theta)
%  cos(theta)
%  theta_dot
%  u]

X_raw = [ ...
    sin_theta, ...
    cos_theta, ...
    theta_dot, ...
    u ...
    ]';


% Targets at time t + 0.01 s
%
% [sin(theta_next)
%  cos(theta_next)
%  theta_dot_next]

Y_raw = [ ...
    sin_theta_n, ...
    cos_theta_n, ...
    theta_dot_n ...
    ]';


%% STEP 3: Train / Validation Split

Tlen = size(X_raw,2);

split_idx = round(0.70 * Tlen);

iTr = 1:split_idx;
iVa = (split_idx + 1):Tlen;


%% STEP 4: Calculate Normalization Using TRAINING DATA ONLY

muX = mean(X_raw(:,iTr), 2);
sigX = std(X_raw(:,iTr), 0, 2) + 1e-9;

muY = mean(Y_raw(:,iTr), 2);
sigY = std(Y_raw(:,iTr), 0, 2) + 1e-9;


%% STEP 5: Normalize Entire Dataset

Xn = (X_raw - muX) ./ sigX;

Yn = (Y_raw - muY) ./ sigY;


%% STEP 6: Build Training and Validation Sequences

Xtr = {Xn(:,iTr)};
Ytr = {Yn(:,iTr)};

Xva = {Xn(:,iVa)};
Yva = {Yn(:,iVa)};


%% STEP 7: Define LSTM Network

inputSize = 4;
outputSize = 3;

layers = [

    sequenceInputLayer(inputSize)

    lstmLayer(96, ...
        'OutputMode','sequence')

    dropoutLayer(0.2)

    lstmLayer(64, ...
        'OutputMode','sequence')

    dropoutLayer(0.2)

    fullyConnectedLayer(outputSize)

    regressionLayer

];


%% STEP 8: Training Options

options = trainingOptions('adam', ...
    'MaxEpochs', 500, ...
    'MiniBatchSize', 1, ...
    'InitialLearnRate', 1e-3, ...
    'Shuffle', 'never', ...
    'ValidationData', {Xva,Yva}, ...
    'ValidationFrequency', 100, ...
    'Verbose', false, ...
    'Plots', 'training-progress');


%% STEP 9: Train Network

disp('Training LSTM dynamics model...');

net = trainNetwork( ...
    Xtr, ...
    Ytr, ...
    layers, ...
    options ...
    );

disp('Training complete.');


%% STEP 10: Teacher-Forced Prediction

% Feed the measured state/control sequence into the network.
% This tests ONE-STEP prediction quality.

Yp_n = predict(net, {Xn});

Yp_n = Yp_n{1};


%% STEP 11: Denormalize Prediction

Yp = Yp_n .* sigY + muY;

Yt = Y_raw;


%% STEP 12: Normalize Predicted sin/cos to Unit Circle

sin_p = Yp(1,:);
cos_p = Yp(2,:);

mag = sqrt( ...
    sin_p.^2 + ...
    cos_p.^2 ...
    ) + eps;

sin_p = sin_p ./ mag;
cos_p = cos_p ./ mag;


%% STEP 13: Reconstruct Theta

theta_true_wrapped = atan2( ...
    Yt(1,:), ...
    Yt(2,:) ...
    );

theta_pred_wrapped = atan2( ...
    sin_p, ...
    cos_p ...
    );


% Continuous angle for visualization

theta_true_unwrapped = ...
    unwrap(theta_true_wrapped);

theta_pred_unwrapped = ...
    unwrap(theta_pred_wrapped);


%% STEP 14: Plot Unwrapped Theta

stride = max(1, floor(Tlen/1000));

idx = 1:stride:Tlen;

t = time(:).';


figure( ...
    'Name', ...
    'Theta Prediction: Ground Truth vs LSTM' ...
    );

plot( ...
    t(idx), ...
    theta_true_unwrapped(idx), ...
    'b', ...
    'LineWidth',1.2 ...
    );

hold on;

plot( ...
    t(idx), ...
    theta_pred_unwrapped(idx), ...
    'r--', ...
    'LineWidth',1.2 ...
    );

legend( ...
    '\theta_{true}', ...
    '\theta_{predicted}', ...
    'Location','best' ...
    );

xlabel('Time (s)');

ylabel('\theta (rad)');

title( ...
    '\theta_{t+1}: Ground Truth vs LSTM Prediction' ...
    );

grid on;


%% STEP 15: Plot Individual State Predictions

figure( ...
    'Name', ...
    'LSTM Next-Step Predictions' ...
    );


% sin(theta)

subplot(2,2,1);

plot( ...
    t(idx), ...
    Yt(1,idx), ...
    'b', ...
    t(idx), ...
    sin_p(idx), ...
    'r--', ...
    'LineWidth',1.1 ...
    );

title('sin(\theta_{t+1})');

legend('True','Predicted');

grid on;


% cos(theta)

subplot(2,2,2);

plot( ...
    t(idx), ...
    Yt(2,idx), ...
    'b', ...
    t(idx), ...
    cos_p(idx), ...
    'r--', ...
    'LineWidth',1.1 ...
    );

title('cos(\theta_{t+1})');

legend('True','Predicted');

grid on;


% theta

subplot(2,2,3);

plot( ...
    t(idx), ...
    theta_true_wrapped(idx), ...
    'b', ...
    t(idx), ...
    theta_pred_wrapped(idx), ...
    'r--', ...
    'LineWidth',1.1 ...
    );

title('\theta_{t+1} (rad)');

legend('True','Predicted');

grid on;


% theta_dot

subplot(2,2,4);

plot( ...
    t(idx), ...
    Yt(3,idx), ...
    'b', ...
    t(idx), ...
    Yp(3,idx), ...
    'r--', ...
    'LineWidth',1.1 ...
    );

title('\theta dot_{t+1} (rad/s)');

legend('True','Predicted');

grid on;


sgtitle( ...
    'Next-Step Teacher-Forced Prediction' ...
    );


%% STEP 16: Calculate Prediction Metrics

theta_error = ...
    theta_pred_wrapped - theta_true_wrapped;

% Wrap angular error into [-pi, pi]
theta_error = ...
    atan2( ...
        sin(theta_error), ...
        cos(theta_error) ...
        );


theta_rmse = sqrt( ...
    mean(theta_error.^2) ...
    );

theta_dot_rmse = sqrt( ...
    mean( ...
        (Yp(3,:) - Yt(3,:)).^2 ...
        ) ...
    );


fprintf('\n=== Teacher-Forced Prediction Performance ===\n');

fprintf( ...
    'Theta RMSE: %.6f rad (%.4f deg)\n', ...
    theta_rmse, ...
    rad2deg(theta_rmse) ...
    );

fprintf( ...
    'Theta-dot RMSE: %.6f rad/s\n', ...
    theta_dot_rmse ...
    );


%% STEP 17: Save Model

save( ...
    'lstmmodel111.mat', ...
    'net', ...
    'muX', ...
    'sigX', ...
    'muY', ...
    'sigY' ...
    );

disp('✅ Model saved as lstm_fdnew_model.mat');
function EnhancedMPCTest_LSTM_MPC
%% Learning-Based MPC using LSTM Dynamics Model
% Corrected version:
% - LSTM sequence history maintained
% - MPC rollout uses same LSTM history
% - Multiple initial angles
% - Higher simulation resolution
% - Trajectory export

clear; clc; close all;


%% ===================== Test Configuration =====================



% Initial angle experiments
% initial_angles_deg = [5 10 15 20 30 40 50 60 70];
initial_angles_deg = [5];
    

%% ===================== Model Loading =====================

% modelFile = 'lstm_fdnew_model.mat';
modelFile = 'lstmmodel111';


fprintf('Loading model: %s\n', modelFile);

S = load(modelFile);

net  = S.net;
muX  = S.muX;
sigX = S.sigX;

muY  = S.muY;
sigY = S.sigY;


fprintf('Model loaded successfully.\n')

fprintf('Control mean:  %.6f\n', muX(4));
fprintf('Control sigma: %.6f\n', sigX(4));


%% ===================== Run Tests =====================


fprintf('\n');
fprintf('====================================\n');
fprintf(' Initial Angle Tests\n');
fprintf('====================================\n');

results_series1 = cell(length(initial_angles_deg),1);

for i = 1:length(initial_angles_deg)

    angle0 = initial_angles_deg(i);
    fprintf('\nTesting initial angle: %d deg\n',angle0);

    results_series1{i}=runSingleTest(...
        angle0,...
        sprintf('Angle_%ddeg',angle0),...
        net,...
        muX,...
        sigX,...
        muY,...
        sigY);

    results_series1{i}.test_label = sprintf('%d deg',angle0);

end


%% ===================== Plot Results =====================


if ~isempty(results_series1)

    generateComparisonPlots(...
        results_series1,...
        'Initial Angle Comparison');

end


fprintf('\nSimulation finished.\n')


end

%% ============================================================
% Single Test Simulation
% ============================================================

function result = runSingleTest(theta0_deg,...
                                test_name,...
                                net,...
                                muX,...
                                sigX,...
                                muY,...
                                sigY)



%% ===================== Simulation Settings =====================

dt = 0.01;          % control timestep
Tfinal = 3;

N = round(Tfinal/dt);

time = (0:N-1)*dt;



%% ===================== LSTM-MPC Parameters =====================


% LSTM sequence length
history_length = 10;


% MPC horizon
H = 30;


% Control limits
u_bounds = [-120 120];


% Cost weights

% w_theta = 5;
% 
% w_theta_dot = 0.5;
% 
% w_u = 1e-4;
w_theta = 40;
w_theta_dot = 20;
w_u = 0.01;

%% ===================== Initial State =====================


theta0 = deg2rad(theta0_deg);


theta_dot0 = 0;


% State:
% x=[sin(theta);cos(theta);theta_dot]

x = [
    sin(theta0);
    cos(theta0);
    theta_dot0
];



%% ===================== Initialize LSTM History =====================


% History:
% row 1: sin(theta)
% row 2: cos(theta)
% row 3: theta_dot
% row 4: control input


history = zeros(4,history_length);


for i=1:history_length

    history(:,i)= [
        x;
        0
    ];

end



%% ===================== Storage =====================


theta_history=zeros(1,N);

theta_dot_history=zeros(1,N);

state_history=zeros(3,N);

u_applied=zeros(1,N);

exit_flags=zeros(1,N);



%% ===================== MPC Loop =====================


prev_U=zeros(H,1);

fprintf('\nStarting simulation: %s\n', test_name);
fprintf('Total steps: %d | MPC Horizon: %d\n', N, H);


for k=1:N




    if mod(k,50)==0

        fprintf('Progress %.1f%% | Simulation time %.2fs | Angle %.2f deg\n',...
            k/N*100,...
            time(k),...
            rad2deg(atan2(x(1),x(2))));

    end



    % Current angle

    theta = atan2(x(1),x(2));


    theta_history(k)=theta;

    theta_dot_history(k)=x(3);

    state_history(:,k)=x;



    % Initial guess

    U0=[prev_U(2:end);0];



    % Solve MPC

    [u_k,Uopt,exitflag]=mpc_step(...
        history,...
        net,...
        muX,...
        sigX,...
        muY,...
        sigY,...
        H,...
        u_bounds,...
        w_theta,...
        w_theta_dot,...
        w_u,...
        U0);



    exit_flags(k)=exitflag;



    % Clamp control

    u_k=min(max(u_k,...
        u_bounds(1)),...
        u_bounds(2));


    u_applied(k)=u_k;



    % Save previous solution

    prev_U=Uopt;



    % Apply learned dynamics

    [x,history]=fd_step_lstm(...
        history,...
        u_k,...
        net,...
        muX,...
        sigX,...
        muY,...
        sigY);




end



%% ===================== Analysis =====================


theta_unwrapped=unwrap(theta_history);



max_deviation=max(abs(rad2deg(theta_unwrapped)));

max_control=max(abs(u_applied));



% settling time

tol=deg2rad(2);

hold_time=1;

hold_N=round(hold_time/dt);


settle_time=NaN;



for k=1:(N-hold_N)

    if all(abs(theta_unwrapped(k:k+hold_N))<tol)

        settle_time=time(k);

        break

    end

end



%% ===================== Save Trajectory =====================


trajectory_table=table(...
    time(:),...
    rad2deg(theta_unwrapped(:)),...
    theta_dot_history(:),...
    state_history(1,:)',...
    state_history(2,:)',...
    u_applied(:),...
    'VariableNames',...
    {
    'Time',...
    'Theta_deg',...
    'Theta_dot',...
    'Sin_theta',...
    'Cos_theta',...
    'Control'
    });



filename=[test_name '_trajectory.csv'];

writetable(trajectory_table,filename);



fprintf('Saved trajectory: %s\n',filename);



%% ===================== Output Struct =====================


result.test_name=test_name;

result.theta0_deg=theta0_deg;

result.time=time;

result.theta_unwrapped=theta_unwrapped;

result.theta_dot=theta_dot_history;

result.state_history=state_history;

result.u_applied=u_applied;

result.max_deviation=max_deviation;

result.max_control=max_control;

result.settle_time=settle_time;

result.exit_flags=exit_flags;




end

%% ============================================================
% MPC Optimization Function
% ============================================================

function [u0,Uopt,exitflag]=mpc_step(...
    history,...
    net,...
    muX,...
    sigX,...
    muY,...
    sigY,...
    H,...
    u_bounds,...
    w_theta,...
    w_theta_dot,...
    w_u,...
    U0)



cost_fun=@(U) rollout_cost(...
    U,...
    history,...
    net,...
    muX,...
    sigX,...
    muY,...
    sigY,...
    H,...
    w_theta,...
    w_theta_dot,...
    w_u);



lb=u_bounds(1)*ones(H,1);

ub=u_bounds(2)*ones(H,1);



options=optimoptions(...
    'fmincon',...
    'Display','off',...
    'MaxIterations',40,...
    'OptimalityTolerance',1e-3,...
    'StepTolerance',1e-3);



try

    [Uopt,~,exitflag]=fmincon(...
        cost_fun,...
        U0,...
        [],[],[],[],...
        lb,...
        ub,...
        [],...
        options);


catch

    Uopt=U0;

    exitflag=-1;

end



% Safety fallback

if exitflag<=0 || isempty(Uopt)

    fprintf('WARNING: MPC failed! exitflag=%d\n',exitflag);

    Uopt=U0;

else

    fprintf('MPC success! exitflag=%d, first control=%.4f\n',...
        exitflag,...
        Uopt(1));

end


u0=Uopt(1);



end

%% ============================================================
% MPC Rollout Cost
% ============================================================

function J=rollout_cost(...
    U,...
    history,...
    net,...
    muX,...
    sigX,...
    muY,...
    sigY,...
    H,...
    w_theta,...
    w_theta_dot,...
    w_u)



J=0;



for k=1:H


    % Predict next state using LSTM

    [x,history]=fd_step_lstm(...
        history,...
        U(k),...
        net,...
        muX,...
        sigX,...
        muY,...
        sigY);



    theta=atan2(x(1),x(2));


    % Cost function

    J=J + ...
        w_theta*(theta^2) + ...
        w_theta_dot*(x(3)^2) + ...
        w_u*(U(k)^2);



end



J=double(J);



end

%% ============================================================
% LSTM Dynamics Prediction Step
% ============================================================

function [x_next,history]=fd_step_lstm(...
    history,...
    u,...
    net,...
    muX,...
    sigX,...
    muY,...
    sigY)



%% Add current control input

history(4,end)=u;



%% Normalize sequence

historyN=(history-muX)./sigX;



%% LSTM prediction

X={historyN};



try

    YN=predict(net,X);

catch

    YN=predict(net,{historyN});

end



if iscell(YN)

    YN=YN{1};

end



% Use final timestep prediction

yN=YN(:,end);



%% Denormalize

y=yN.*sigY+muY;



%% Keep sin/cos on unit circle

mag=sqrt(y(1)^2+y(2)^2)+eps;


y(1)=y(1)/mag;

y(2)=y(2)/mag;



%% New state

x_next=double(y(:));



%% Update history buffer

history(:,1:end-1)=history(:,2:end);



history(:,end)=...
    [x_next;0];



end

%% ============================================================
% Plot Comparison Results
% ============================================================

function generateComparisonPlots(results,series_title)


if isempty(results)
    return
end


n_tests=length(results);


figure(...
    'Name',series_title,...
    'Position',[100 100 1400 900]);



%% ============================================================
% 1. Angle Response
% ============================================================


subplot(2,2,1)

hold on


for i=1:n_tests

    r=results{i};

    plot(...
        r.time,...
        rad2deg(r.theta_unwrapped),...
        'LineWidth',1.5,...
        'DisplayName',r.test_label);

end



yline(0,'k--',...
    'DisplayName','Target');


yline(2,'g:',...
    'DisplayName','±2 deg');


yline(-2,'g:',...
    'HandleVisibility','off');



xlabel('Time (s)')

ylabel('\theta (deg)')

title('Angle Response')

grid on

legend('Location','best')





%% ============================================================
% 2. Control Input
% ============================================================


subplot(2,2,2)

hold on



for i=1:n_tests

    r=results{i};


    plot(...
        r.time,...
        r.u_applied,...
        'LineWidth',1.5,...
        'DisplayName',r.test_label);

end



xlabel('Time (s)')

ylabel('Control Input u')

title('Control Effort')

grid on

legend('Location','best')





%% ============================================================
% 3. Performance Metrics
% ============================================================


subplot(2,2,3)


max_dev=zeros(n_tests,1);

settling=zeros(n_tests,1);



labels=cell(n_tests,1);



for i=1:n_tests

    r=results{i};


    max_dev(i)=r.max_deviation;


    settling(i)=r.settle_time;


    labels{i}=r.test_label;


end



yyaxis left

bar(max_dev)

ylabel('Maximum deviation (deg)')


yyaxis right

bar(settling)

ylabel('Settling time (s)')


set(gca,...
    'XTick',1:n_tests,...
    'XTickLabel',labels)



xlabel('Initial condition')


title('Performance Comparison')


grid on





%% ============================================================
% 4. Phase Portrait
% ============================================================


subplot(2,2,4)

hold on



for i=1:n_tests


    r=results{i};


    plot(...
        rad2deg(r.theta_unwrapped),...
        r.theta_dot,...
        'LineWidth',1.5,...
        'DisplayName',r.test_label);


end



plot(0,0,...
    'ko',...
    'MarkerSize',8,...
    'DisplayName','Equilibrium');



xlabel('\theta (deg)')

ylabel('\theta dot (rad/s)')


title('Phase Portrait')


grid on

legend('Location','best')



sgtitle(series_title,...
    'FontSize',14,...
    'FontWeight','bold')



end


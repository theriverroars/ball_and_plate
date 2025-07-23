% Parameters

clear all; close all; clc;

% Ball Plate Parameters
m = 0.5;                % Mass of the ball (kg)
r = 0.05;               % Radius of the ball (m)
g = 9.81;               % Gravitational acceleration (m/s^2)
I = (2/5) * m * r^2;    % Moment of inertia of the ball (kg*m^2)

const_coeff = (m*g/(m + I/r^2)); % Constant in System Dynamics

% Trajectory parameters
r = 0.25;         % Radius of the trajectory (m)
omega = 0.50;     % Angular frequency (rad/s)


%%

%LQR

% Define linearised state-space model
A = [0, 1, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 1;
     0, 0, 0, 0];

B = [0,        0;
    -5/7 * g, 0;
     0,        0;
     0, -5/7 * g];

% Define LQR weighting matrices
Q = diag([10, 1, 10, 1]); % State weighting matrix
R = diag([1, 1]);         % Control input weighting matrix

% Calculate the LQR gain matrix
K_lqr = lqr(A, B, Q, R);


%%

% PID

% Time Step
dt = 0.01; 

% Setting the PID gains
K_pid = [0.2, 1., 0.25]; % KI, KP, KD


%%

% SMC

% Setting the SMC gains
% Sigma_K, Sigma_delta, S_n1, S_n2, 1 
K_smc = [2.5, 1.0, 3.85, 4.40, 1];


%%

% Define initial state and
% simulation duration
ti = 0; tf = 20;
X0 = [0; 0; 0; 0];
tspan = ti : dt : tf;

% Choosing the Trajectory
curr_traj = @(t) circularTrajectory(t, r, omega);
% curr_traj = @(t) fig8_Trajectory(t, r, omega);

% Choosing the respective controller
% xdot_ = @(t, x) xdot_lqr(t, x, const_coeff, K_lqr, curr_traj);
% xdot_ = @(t, x) xdot_pid(t, x, const_coeff, K_pid, dt, curr_traj);
xdot_ = @(t, x) xdot_smc(t, x, const_coeff, K_smc, dt, curr_traj);

% Simulating the closed-loop system
U = zeros(length(tspan), 2);
X_actual = zeros(length(tspan), 4);
X_reference = zeros(length(tspan), 4);

for i = 1:length(tspan)
    [x_dot_curr, U(i,:), X_reference(i,:)] = xdot_(tspan(i), X_actual(i,:));
    X_actual(i+1,:) = X_actual(i,:) + x_dot_curr'*dt;
end

%%

% Inverse Kinematics
% D = 0.1;
% Rm = 0.03;
% rb = 0.75;
% r_pl = 0.1;
% rd  = 0.01;
% Tb = [0 0 0.07];

D = 0.23;
 Rm = 0.08; 
 rb = 0.17;
 r_pl = 0.15;
 rd = 0.05;
 Tb= [0 0 0.22];

r= deg2rad([60 120 180 240 300 360]);
rt= deg2rad([90 90 240 240 300 300]);
rp= deg2rad([45 135 165 255 285 15]);
rt_ = deg2rad([180 0 300 120 60 240]);
%or  = [-1 +1 -1 +1 -1 +1]

B = zeros(6,3); 
B(:,1) = rb*cos(r)+ rd*cos(rt);
B(:,2) = rb*sin(r)+ rd*sin(rt);

P_p = zeros(6,3);
P_p(:,1) = r_pl*cos(rp);
P_p(:,2) = r_pl*sin(rp);

delta = zeros(length(tspan), 6);
modified_inv_kin = @(alpha, beta) inv_(alpha, beta, B, P_p, Tb, Rm, D, rt_);

for i = 1:length(tspan)
    delta(i,:) = modified_inv_kin(U(i,1), U(i,2))';
end


%%

% Plot results
figure;
hold on;
plot(X_actual(:,1),X_actual(:,3), 'r');
plot(X_reference(:,1), X_reference(:,3), 'b--');
hold off;

xlabel('X (m)'); ylabel('Y (m)');
legend('Actual', 'Reference');
axis equal; grid on;
title('Actual Vs Reference Trajectories');

% Plot results
figure;
plot(tspan, rad2deg(U));

xlabel('Time (s)'); ylabel('Control inputs (deg)');
legend('Alpha (deg)', 'Beta (deg)');
title('Plate Angles');
grid on;

% Plot results
figure;
error = X_reference - X_actual(1:end-1,:);
plot(tspan, error(:,1), tspan, error(:,3));

xlabel('Time (s)'); ylabel('Error');
legend('X (m)', 'Y (m)');
title('Error in Position');
grid on;

% Plot results
figure;
plot(tspan, delta)
xlabel('Time (s)'); ylabel('Join Angles (deg)');
title('Joint Angles Variation');
grid on;
%% Calculating performance metrics for X and Y positions

% Defining acceptable error band for settling time (2% band)
error_band = 0.02; 

% Extract error in X and Y positions
error_X = error(:, 1); % X-axis error
error_Y = error(:, 3); % Y-axis error

% Final values (for steady-state error calculation)
final_value_X = X_reference(end, 1);
final_value_Y = X_reference(end, 3);

% Steady-State Error
ss_error_X = abs(final_value_X - X_actual(end-1, 1));
ss_error_Y = abs(final_value_Y - X_actual(end-1, 3));

% Rise Time (10% to 90% of final value in position)
rise_time_X = find(abs(error_X) >= 0.1*final_value_X & abs(error_X) <= 0.9*final_value_X, 1, 'last') * dt;
rise_time_Y = find(abs(error_Y) >= 0.1*final_value_Y & abs(error_Y) <= 0.9*final_value_Y, 1, 'last') * dt;

% Settling Time (time to remain within 2% band of final value)
settling_time_X = find(abs(error_X) <= error_band*abs(final_value_X), 1, 'last') * dt;
settling_time_Y = find(abs(error_Y) <= error_band*abs(final_value_Y), 1, 'last') * dt;

% Display results
fprintf('Performance metrics for X position:\n');
fprintf('Rise Time: %.2f s\n', rise_time_X);
fprintf('Settling Time: %.2f s\n', settling_time_X);
fprintf('Steady-State Error: %.4f m\n', ss_error_X);

fprintf('\nPerformance metrics for Y position:\n');
fprintf('Rise Time: %.2f s\n', rise_time_Y);
fprintf('Settling Time: %.2f s\n', settling_time_Y);
fprintf('Steady-State Error: %.4f m\n', ss_error_Y);




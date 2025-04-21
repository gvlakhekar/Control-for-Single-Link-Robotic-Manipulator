% -------------------------------------------------------------------------
% Neural network-based trajectory tracking for pendulum
%--------------------------------------------------------------------------
clc; clear; close all;
%--------------------------------------------------------------------------
% Neural Network for Single Link Robotic Manipulator  
% (Prepared By: Dr. Girish V. Lakhekar)
%--------------------------------------------------------------------------
M = 1; % Mass of link (kg)
g = 9.81; % Gravitational Constant (m/s^2)
l = 1;  % Link Length  (m)
B = 1;  % Damping Factor (kgm^2/s)
J = 1;  % Total moment of inertia (kgm^2)
dt = 0.02; T = 10;
steps = T / dt;
time = 0:dt:T-dt;

% Desired trajectory: sine wave
theta_ref = 0.5 * sin(2 * pi * 0.2 * time);         % Desired angle
theta_dot_ref = 0.5 * 2 * pi * 0.2 * cos(2 * pi * 0.2 * time);  % Desired angular velocity

% PD gains (used for data generation)
Kp = 50; Kd = 8;
%% -----------------------
% GENERATE TRAINING DATA

X = []; Y = [];

for i = 1:steps
    % Simulate ideal PD controller
    theta = theta_ref(i) + 0.2 * randn();  % Add noise for generalization
    theta_dot = theta_dot_ref(i) + 0.2 * randn();

    e = theta - theta_ref(i);
    edot = theta_dot - theta_dot_ref(i);

    u = -Kp * e - Kd * edot;  % Target torque

    input = [theta; theta_dot; theta_ref(i); theta_dot_ref(i)];
    X = [X input];
    Y = [Y u];
end

%% -----------------------
% TRAIN NEURAL NETWORK
%-------------------------

net = feedforwardnet([10 10]);  % Two hidden layers
net.trainParam.showWindow = true;
net = train(net, X, Y);

%% ----------------------------
% SIMULATION WITH NN CONTROLLER
%-------------------------------

theta = pi / 3; theta_dot = 0;
theta_hist = zeros(1, steps);

for i = 1:steps
    % Desired trajectory at time t
    theta_d = theta_ref(i);
    theta_dot_d = theta_dot_ref(i);

    % NN input: current + desired state
    input = [theta; theta_dot; theta_d; theta_dot_d];
    u = net(input);  % Control torque

    % Dynamic Model of Single Link Robotic Manipulator
    theta_ddot = (-B/J)*theta_dot-(M*g*l)/J*sin(theta) + u;

    % Euler integration
    theta_dot = theta_dot + theta_ddot * dt;
    theta = theta + theta_dot * dt;

    theta_hist(i) = theta;
    u_hist(i) = u;
    theta_dot_hist(i) = theta_dot;
end
%% -----------------------
% PLOT RESULTS
%%-------------------------
figure;
plot(time, theta_ref, '-.r', 'LineWidth', 1.5); hold on;
plot(time, theta_hist,'b','LineWidth', 1.5);
xlabel('Time (s)','fontsize',12,'interpreter','latex');
ylabel('Joint Angle (rad)','fontsize',12,'interpreter','latex');
legend('Reference Trajectory', 'Joint Angle','fontsize',12,'interpreter','latex');
title('Trajectory Tracking using Neural Network','fontsize',12,'interpreter','latex');
grid on;
figure
plot(time,(theta_ref-theta_hist),'k','LineWidth', 1.5)
hold on
plot(time,(theta_dot_ref-theta_dot_hist),':m','LineWidth', 1.5)
grid on
title('Joint Positional Error $(\theta_{e})$ and Velocity Error $(\dot{\theta}_{e})$','fontsize',12,'interpreter','latex');
xlabel('Time (s)','fontsize',12,'interpreter','latex'); ylabel('$\theta_{e}$ (rad) and $\dot{\theta}_{e}$ (rad/s)','fontsize',12,'interpreter','latex');
legend('$\theta_{e}$','$\dot{\theta}_{e}$','fontsize',12,'interpreter','latex')
figure
plot(time,u_hist,'k', 'LineWidth', 1.5)
grid on
title('Joint Torque $(\tau)$','fontsize',12,'interpreter','latex');
xlabel('Time (s)','fontsize',12,'interpreter','latex'); ylabel('Torque (Nm)','fontsize',12,'interpreter','latex');

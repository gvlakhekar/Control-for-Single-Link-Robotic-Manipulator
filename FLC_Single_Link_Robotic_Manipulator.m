%--------------------------------------------------------------------------
% Fuzzy Logic Control for Single Link Robotic Manipulator  
% (Prepared By: Dr. Girish V. Lakhekar)
%---------------------------------------------------------------------------
clear all;
clc;
M = 1; % Mass of link (kg)
g = 9.81; % Gravitational Constant (m/s^2)
l = 1;  % Link Length  (m)
B = 1;  % Damping Factor (kgm^2/s)
J = 1;  % Total moment of inertia (kgm^2)
x1(1,1) = 0; % Initial Joint Angle 
x2(1,1) = 0; % Initial Joint 
x1d = pi/4; % Desired Joint Angle
x2d = 0; % Desired Joint Velocity
T=500; % Time Period
error_int = zeros(T,1);
dt = 0.01; % Stepsize

%% PID Base Gains % 70 80 2 sine % 7 40 2 Step
Kp0 = 7; 
Ki0 = 40;
Kd0 = 2; 

%% Define Fuzzy Inference System
fis = mamfis('Name','FuzzyPID');

% Inputs
fis = addInput(fis, [-1 1], 'Name', 'error');
fis = addInput(fis, [-1 1], 'Name', 'd_error');

% Outputs: dKp, dKi, dKd
fis = addOutput(fis, [-2 2], 'Name', 'dKp');
fis = addOutput(fis, [-1 1], 'Name', 'dKi');
fis = addOutput(fis, [-1 1], 'Name', 'dKd');

% Membership Functions
mf_vals = [-1 -0.5 0; -0.5 0 0.5; 0 0.5 1];
mf_names = {'N','Z','P'};

for i = 1:3
    fis = addMF(fis, 'error', 'trimf', mf_vals(i,:), 'Name', mf_names{i});
    fis = addMF(fis, 'd_error', 'trimf', mf_vals(i,:), 'Name', mf_names{i});
end

fis = addMF(fis, 'dKp', 'trimf', [-2 -1 0], 'Name', 'N');
fis = addMF(fis, 'dKp', 'trimf', [-1 0 1], 'Name', 'Z');
fis = addMF(fis, 'dKp', 'trimf', [0 1 2], 'Name', 'P');

fis = addMF(fis, 'dKi', 'trimf', [-1 -0.5 0], 'Name', 'N');
fis = addMF(fis, 'dKi', 'trimf', [-0.5 0 0.5], 'Name', 'Z');
fis = addMF(fis, 'dKi', 'trimf', [0 0.5 1], 'Name', 'P');

fis = addMF(fis, 'dKd', 'trimf', [-1 -0.5 0], 'Name', 'N');
fis = addMF(fis, 'dKd', 'trimf', [-0.5 0 0.5], 'Name', 'Z');
fis = addMF(fis, 'dKd', 'trimf', [0 0.5 1], 'Name', 'P');

% Rules
ruleList = [
    1 1 3 1 1 1 1;
    1 2 2 2 2 1 1;
    1 3 2 1 1 1 1;
    2 1 3 2 2 1 1;
    2 2 2 3 2 1 1;
    2 3 2 2 3 1 1;
    3 1 2 1 3 1 1;
    3 2 1 2 2 1 1;
    3 3 1 1 1 1 1;
];

fis = addRule(fis, ruleList);

for i = 1:T
    t(i)=dt*i;
    x1d(i,1)= pi/4;
    x2d(i,1)= 0;

    %x1d(i,1) = 0.5 * sin(2 * pi * 0.2 * t(i));         % Desired angle
    %x2d(i,1) = 0.5 * 2 * pi * 0.2 * cos(2 * pi * 0.2 * t(i));  % Desired angular velocity

    e(i,1) = (x1d(i,1)-x1(i,1));
    edot(i,1) = (x1d(i,1)-x2(i,1));
    error_int(i,1) = error_int(i,1) + e(i,1)* dt;

    % Normalize inputs
    e_norm(i,1) = max(-1, min(1, e(i,1)));
    de_norm(i,1) = max(-1, min(1, edot(i,1)));

    % Fuzzy PID Tuning
    dGains = evalfis(fis, [e_norm(i,1), de_norm(i,1)]);
    Kp = Kp0 + dGains(1);
    Ki = Ki0 + dGains(2);
    Kd = Kd0 + dGains(3);

    % Fuzzy PID Control Law
    u(i,1) = Kp * e(i,1) + Ki * error_int(i,1) + Kd * edot(i,1);
   
if(u(i,1)>20)
    u(i,1)=20;
elseif (u(i,1)<-20)
    u(i,1)=-20;
end
x1(i+1,1) = x1(i,1)+dt*(x2(i,1));
x2(i+1,1) = x2(i,1)+dt*(-B/J*x2(i,1)-(M*g*l)/J*sin(x1(i,1))+3.5*u(i,1)); %3.5 step 5.5 sine
end
figure
subplot(3,1,1)
plot(t,x1d,'--r','LineWidth', 1.5)
hold on
plot(t,x1(1:end-1),'b','LineWidth', 1.5)
grid on
title('Joint Angle $(\theta)$','fontsize',12,'interpreter','latex');
xlabel('Time (s)','fontsize',12,'interpreter','latex'); ylabel('Angle (rad)','fontsize',12,'interpreter','latex');
subplot(3,1,2)
plot(t,x2(1:end-1),'g', 'LineWidth', 1.5) 
grid on
title('Joint Velocity $(\dot{\theta})$','fontsize',12,'interpreter','latex');
xlabel('Time (s)','fontsize',12,'interpreter','latex'); ylabel('Velocity (rad/s)','fontsize',12,'interpreter','latex');
subplot(3,1,3)
plot(t,u,'m', 'LineWidth', 1.5)
grid on
title('Joint Torque $(\tau)$','fontsize',12,'interpreter','latex');
xlabel('Time (s)','fontsize',12,'interpreter','latex'); ylabel('Torque (Nm)','fontsize',12,'interpreter','latex');
figure
plot(t,e,'k','LineWidth', 1.5)
hold on
plot(t,edot,'Color','#A2142F','LineWidth', 1.5)
grid on
title('Joint Positional Error $(\theta_{e})$ and Velocity Error $(\dot{\theta}_{e})$','fontsize',12,'interpreter','latex');
xlabel('Time (s)','fontsize',12,'interpreter','latex'); ylabel('$\theta_{e}$ (rad) and $\dot{\theta}_{e}$ (rad/s)','fontsize',12,'interpreter','latex');
legend('$\theta_{e}$','$\dot{\theta}_{e}$','fontsize',12,'interpreter','latex')
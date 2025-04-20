%--------------------------------------------------------------------------
% Sliding Mode Control for Single Link Robotic Manipulator
% (Prepared By: Dr. Girish V. Lakhekar)
%--------------------------------------------------------------------------
M = 1; % Mass of link (kg)
g = 9.81; % Gravitational Constant (m/s^2)
l = 1;  % Link Length  (m)
B = 1;  % Damping Factor (kgm^2/s)
J = 1;  % Total moment of inertia (kgm^2)
x1(1,1) = 0; % Initial Joint Angle 
x2(1,1) = 0; % Initial Joint 
dt = 0.01; % Stepsize
la = 5;  % Sliding Surface Coefficient
Kd = 10;   % Hitting Gain
epsilon = 0.1;

for i = 1:500
    t(i)=dt*i;
    x1d(i,1)= pi/4;
    x2d(i,1)= 0;
    x2d_ddot(i,1) = 0;
    e(i,1) = (x1(i,1)- x1d(i,1));
    edot(i,1) = (x2(i,1)-x2d(i,1));
    s(i,1) = la*e(i,1)+ edot(i,1);
    sat_s(i,1) = s(i,1) / epsilon;
    if abs(sat_s(i,1)) > 1
        sat_s(i,1) = sign(sat_s(i,1));
    end
     u(i,1) = J*(x2d_ddot(i,1) - la*edot(i,1)) + B*x2(i,1) + M*g*l*sin(x1(i,1)) - Kd*sat_s(i,1);
if(u(i,1)>20)
     u(i,1)=20;
elseif (u(i,1)<-20)
    u(i,1)=-20;
end
x1(i+1,1) = x1(i,1)+dt*(x2(i,1));
x2(i+1,1) = x2(i,1)+dt*(-B/J*x2(i,1)-(M*g*l)/J*sin(x1(i,1))+u(i,1)/J);
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
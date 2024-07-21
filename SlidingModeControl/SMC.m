clc, clear, close all
% Sliding Mode Control for Nonlinear System
% Written By: Rasit
% Date: 21-Jul-2024
%%
k = 2.25; % Controller Gain
lambda1 = 1; % Sliding Surface Param
T = 10;    % Sim Time
Ts = 1e-2; % Sampling Time
t = 0:Ts:T;

x1 = 1;
x2 = 1;
omega = 1; % Angular Frequency
A = 1.5;   % Magnitude
r = A*sin(omega*t); % Referance Signal

x1_vec = zeros(size(t));
x2_vec = zeros(size(t));
u_vec = zeros(size(t));
e_vec = zeros(size(t));
edot_vec = zeros(size(t));
s_vec = zeros(size(t));

figure('units', 'normalized', 'outerposition', [0 0 1 1], 'color', 'w')
for i = 1:length(t)
    e = x1-r(i); % Error
    e_dot = x2-A*omega*cos(omega*t(i)); % Error dot

    s = (lambda1*e)+(e_dot); % Sliding Surface
    u = -(k*sign(s)); % Control Signal

    x1_dot = x2;
    x2_dot = -x1+(1-x1^2)*x2+u;
    x1 = x1+x1_dot*Ts; % Euler Integration
    x2 = x2+x2_dot*Ts; % Euler Integration

    x1_vec(i) = x1;
    x2_vec(i) = x2;
    u_vec(i) = u;
    e_vec(i) = e;
    edot_vec(i) = e_dot;
    s_vec(i) = s;

    if mod(i, 10) == 0
        clf
        subplot(221);
        plot(t(1:i), r(1:i), 'r--', 'LineWidth', 3);
        hold on;
        plot(t(1:i), x1_vec(1:i), 'b', 'LineWidth', 2);
        title('Reference Signal and System State x_1');
        xlabel('Time [s]');
        legend('Reference Signal', 'x_1');

        subplot(223);
        plot(t(1:i), x2_vec(1:i), 'LineWidth', 2);
        title('System State x_2');
        xlabel('Time [s]');
        ylabel('x_2');

        subplot(222);
        plot(e_vec(1:i), edot_vec(1:i), 'LineWidth',1.5), hold on
        e_range = linspace(-1,1,100);
        plot(e_range,-lambda1*e_range, 'r', 'LineWidth', 3) % Sliding Surface
        yline(0), xline(0), axis([-.2 1.2 -1 .2])
        scatter(e_vec(1), edot_vec(1), "red", "filled")
        xlabel('$e$', 'Interpreter', 'latex');
        ylabel('$\dot{e}$', 'Interpreter', 'latex');
        legend('$e - \dot{e}$', 'Sliding Surface','Interpreter','latex');

        subplot(224);
        plot(t(1:i), x1_vec(1:i) - r(1:i), 'LineWidth', 2);
        title('Tracking Error');
        xlabel('Time [s]');
        ylabel('Error');
        drawnow
    end
end

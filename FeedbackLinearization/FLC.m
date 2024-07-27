clc,clear,close all;
% Feedback Linearization Control for Nonlinear System
% Written By: Rasit
% Date: 27-Jul-2024
%%
Kp = 15;  % P Gain
Ki = 1;   % I Gain
Kd = .1;  % D Gain

T = 20;      % Simulation Time
Ts = 1e-2;   % Sampling Time
t = 0:Ts:T;  % Time Span

x = zeros(1, length(t)); % State vector
u = zeros(1, length(t)); % Control input
v = zeros(1, length(t)); % New control input
y = zeros(1, length(t)); % Output
e = zeros(1, length(t)); % Error
e_int = 0;    % Error integral
e_prev = 0;   % Previous error

% Reference Signal
r = [sin(t(1:end/2)) ones(1, (size(t,2)/4) - .25) zeros(1, (size(t,2)/4) + .75)];

figure('units', 'normalized', 'outerposition', [0 0 1 1], 'color', 'w')
for k = 1:length(t)-1
    e(k) = r(k)-y(k);
    e_int = e_int+e(k)*Ts;
    e_der = (e(k)-e_prev)/Ts;

    v(k) = Kp*e(k)+Ki*e_int+Kd*e_der;
    u(k) = v(k)+x(k)^3;

    x(k+1) = x(k)+Ts*(-x(k)^3+u(k));  % Nonlinear State Space: State Update
    y(k+1) = x(k+1);
    e_prev = e(k);

    if mod(k,10) == 0
        clf
        subplot(311);
        plot(t, r, '--r', 'LineWidth', 2); hold on;
        plot(t(1:k), y(1:k), 'b', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('y(t)');
        legend('Reference', 'Output');
        title('Feedback Linearization Controller');

        subplot(312);
        plot(t(1:k), u(1:k), 'k', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('u(t)');
        title('Control Input');

        subplot(313);
        plot(t(1:k), e(1:k), 'm', 'LineWidth', 2);
        xlabel('Time (s)');
        ylabel('e(t)');
        title('Error');
        drawnow
    end
end



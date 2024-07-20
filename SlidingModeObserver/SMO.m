clc,clear all,close all;
% LTI System - Sliding Mode Observer
% Written By: Rasit
% Date: 20-Jul-2024 
%% LTI System State Space
A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0;

L = [1; 1]; % Observer Gains

Ts = 5e-2;         
T = 10; 
t = 0:Ts:T;     % Time Vec
N = length(t);  % Number Of Data

x = [1; 0];     % Initial State
x_hat = [0; 0]; % Observer Initial State

X = zeros(2, N);
X_hat = zeros(2, N);
Y = zeros(1, N);
Y_hat = zeros(1, N);

figure('units', 'normalized', 'outerposition', [0 0 1 1], 'color', 'w')
for k = 1:N
    y = C*x;
    y_hat = C*x_hat;
    S = y-y_hat;  % Sliding Surface!

    % Update Observer State
    x_hat_dot = A*x_hat+B*0+L.*sign(S);
    x_hat = x_hat+x_hat_dot*Ts;  % Euler Integration

    % System State Update
    x_dot = A*x+B*0; 
    x = x+x_dot*Ts;  % Euler Integration

    X(:, k) = x;
    X_hat(:, k) = x_hat;
    Y(k) = y;
    Y_hat(k) = y_hat;

    if mod(k,10) == 0
        clf
        subplot(121);
        plot(t(1:k), X(1,1:k),"k",LineWidth=3),hold on
        plot(t(1:k),X_hat(1,1:k),"r",LineWidth=.2)
        legend('Real State x1', 'SMO State x1');
        yline(0,'DisplayName',""),xlabel("Time [Sn]"),axis([0 10 -.6 1.1])
        subplot(122);
        plot(t(1:k), X(2,1:k),"k",LineWidth=3),hold on
        plot(t(1:k),X_hat(2,1:k),"r",LineWidth=.2)
        legend('Real State x2', 'SMO State x2');
        yline(0,'DisplayName',""),xlabel("Time [Sn]"),axis([0 10 -.6 1.1])
        drawnow
    end
end

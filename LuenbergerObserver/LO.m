% Durum-uzay modeli parametreleri
A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = [0];

% Gözlemci kazancı (L matrisi)
L = place(A', C', [-2 -3])';

% Simülasyon parametreleri
dt = 0.01; % Zaman adımı
T = 10; % Toplam simülasyon süresi
t = 0:dt:T; % Zaman vektörü
n = length(t); % Zaman vektörünün uzunluğu

% Başlangıç koşulları
x = [0; 0]; % Gerçek durum vektörü
x_hat = [1; 1]; % Gözlemcinin tahmin ettiği durum vektörü

% Durum ve çıkış vektörlerinin depolanması
X = zeros(2, n);
X_hat = zeros(2, n);
Y = zeros(1, n);

% Simülasyon döngüsü
for i = 1:n
    u = sin(t(i)); % Giriş sinyali (örnek olarak sinüs dalgası)
    y = C*x+D*u; % Çıkış sinyali
    y_hat = C*x_hat+D*u;

    % Gerçek sistem dinamiği
    x_dot = A*x+B*u;
    x = x+x_dot*dt;
    
    % Gözlemci dinamiği
    x_hat_dot = A*x_hat+B*u+L*(y-y_hat);
    x_hat = x_hat+x_hat_dot*dt;
    
    % Durum ve çıkış vektörlerinin kaydedilmesi
    X(:, i) = x;
    X_hat(:, i) = x_hat;
    Y(i) = y;
end

% Sonuçların grafiği
figure;
subplot(121);
plot(t, X(1,:), t, X_hat(1,:));
title('Durum 1');
legend('Gerçek', 'Tahmin');
xlabel('Zaman (s)');
ylabel('Durum 1');

subplot(122);
plot(t, X(2,:), t, X_hat(2,:));
title('Durum 2');
legend('Gerçek', 'Tahmin');
xlabel('Zaman (s)');
ylabel('Durum 2');



clc; clear; close all;

%% 1. Paramètres
dt = 1; % pas de temps

A = [1 dt;
     0 1];     % matrice dynamique

H = [1 0];     % on mesure seulement la position

Q = [0.01 0;
     0 0.01];  % bruit du modèle

R = 1;         % bruit de mesure

%% 2. Initialisation
X = [0; 1];    % état initial [position; vitesse]
P = eye(2);    % matrice de covariance

%% 3. Simulation
N = 50;                % nombre d'itérations
x_true = zeros(1,N);   % vraie position
z = zeros(1,N);        % mesure bruitée
x_est = zeros(1,N);    % estimation Kalman

x = 0; % position réelle
v = 1; % vitesse réelle

for k = 1:N
    
    %%  Système réel 
    x = x + v*dt;
    x_true(k) = x;
    
    %%  Mesure bruitée 
    z(k) = x + randn*1;
    
    %%  Filtre de Kalman 
    
    % 1. Prédiction
    X_pred = A * X;
    P_pred = A * P * A' + Q;
    
    % 2. Gain de Kalman
    K = P_pred * H' / (H * P_pred * H' + R);
    
    % 3. Correction
    X = X_pred + K * (z(k) - H * X_pred);
    P = (eye(2) - K * H) * P_pred;
    
    % Sauvegarde position estimée
    x_est(k) = X(1);
end

%% 4. Affichage des résultats
figure;
plot(x_true, 'g', 'LineWidth',2); hold on;
plot(z, 'r.');
plot(x_est, 'b', 'LineWidth',2);

legend('Position réelle','Mesure bruitée','Estimation Kalman');
title('Filtre de Kalman - Mobile 1D');
xlabel('Temps');
ylabel('Position');
grid on;
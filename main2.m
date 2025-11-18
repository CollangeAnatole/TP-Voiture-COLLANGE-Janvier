%% MAIN - Comparaison des modèles véhicule
clear; close all; clc;

%% Paramètres véhicule
ParaV.masse = 1200;     % kg
ParaV.Iz    = 2500;     % kg.m^2
ParaV.Lf    = 1.2;      % m
ParaV.Lr    = 1.3;      % m
ParaV.Cf    = 80000;    % N/rad
ParaV.Cr    = 80000;    % N/rad

%% Paramètres scénario
% Exemple : accélération + virage
ParaS = creerSim(50, ...              % vmax en km/h
                 10, ...              % braquage max en degrés
                 2, ...               % accélération max en m/s²
                 'acceleration', ...  % scénario vitesse
                 'virage');           % scénario braquage

%% Simulation avec modèle dynamique
[t,u,beta,delta,d_teta,teta,x,y] = model_dynamique(ParaV,ParaS);

%% Simulation avec modèle dynamique 2
[t2,u2,beta2,delta2,d_teta2,teta2,x2,y2] = model_dynamique2(ParaV,ParaS);

%% Simulation avec modèle cinématique
[t3,u3,beta3,delta_f,delta_r,r,r_ng,ay,X,Y] = modele_cinematique(ParaV,ParaS);

%% --- Affichage des trajectoires ---
figure;
plot(x,y,'b','LineWidth',1.5); hold on;
plot(x2,y2,'r--','LineWidth',1.5);
plot(X,Y,'g-.','LineWidth',1.5);
xlabel('x (m)'); ylabel('y (m)');
legend('Modèle Dynamique','Modèle Dynamique 2','Modèle Cinématique');
title('Comparaison des trajectoires');
axis equal; grid on;

%% --- Affichage des vitesses ---
figure;
plot(t,u,'b','LineWidth',1.5); hold on;
plot(t2,u2,'r--','LineWidth',1.5);
plot(t3,u3,'g-.','LineWidth',1.5);
xlabel('Temps (s)'); ylabel('Vitesse (m/s)');
legend('Modèle Dynamique','Modèle Dynamique 2','Modèle Cinématique');
title('Comparaison des vitesses');
grid on;

%% --- Affichage des braquages ---
figure;
plot(t,beta,'b','LineWidth',1.5); hold on;
plot(t2,beta2,'r--','LineWidth',1.5);
plot(t3,beta3,'g-.','LineWidth',1.5);
xlabel('Temps (s)'); ylabel('Braquage (rad)');
legend('Modèle Dynamique','Modèle Dynamique 2','Modèle Cinématique');
title('Comparaison des braquages');
grid on;

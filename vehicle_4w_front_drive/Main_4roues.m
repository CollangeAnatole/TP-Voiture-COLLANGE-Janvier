% Main_4roues.m - exécution modèle 4 roues traction avant
clear; close all; clc;

% --- créer véhicule ---
m = 1310; Iz = 1760; lf = 1.2; lr = 1.4; h = 0.45; Rw = 0.30;
Ca_tire_f = 80000; Ca_tire_r = 70000; mu = 0.9;
CdA = 0.5 * 1.225 * 0.30; % approximatif
Crr = 0.015;
V1 = creerVehicule4(m, Iz, lf, lr, h, Rw, Ca_tire_f, Ca_tire_r, mu, CdA, Crr);

% --- simulation ---
Tf = 20; dt = 0.01;
beta_fun = @(t) (t<2).* (5*(t/2)) + (t>=2).*5; % deg : rampe 0->5deg en 2s
drive_fun = @(t) (t<1).* 120 .* t + (t>=1).*120; % N.m total aux roues avant

S1 = creerSim(100, beta_fun, Tf, dt, drive_fun); % 100 km/h initial

% pneus simples
P = creerPneumatique_simple(40000,35000,8000);

% --- lancer modèles ---
[t_c, Xc, Yc, psic, rc, deltac] = modele_cinematique_4roues(V1, S1);
[t, U, V, r, X, Y, psi, Fx_f, Fy_f, Fy_r] = model_dynamique_4roues(V1, S1, P);

% --- tracés comparatifs ---
figure(1);
plot(Xc, Yc, 'r-', 'LineWidth',2); hold on;
plot(X, Y, 'b--', 'LineWidth',1.5);
legend('Cinématique','Dynamique');
xlabel('x [m]'); ylabel('y [m]'); title('Trajectoire comparée'); axis equal; grid on;

figure(2);
subplot(3,1,1);
plot(t, U, 'b'); hold on;
plot(t_c, S1.v0*ones(size(t_c)), 'r--'); ylabel('U [m/s]'); legend('Dynamique','Consigne');
grid on;

subplot(3,1,2);
plot(t, V, 'b'); ylabel('V [m/s]'); grid on;

subplot(3,1,3);
plot(t, r*180/pi, 'b'); hold on;
plot(t_c, rc*180/pi, 'r--'); ylabel('r [°/s]'); legend('Dynamique','Cinématique'); grid on;

figure(3);
plot(t, Fx_f, 'k', t, Fy_f, 'b', t, Fy_r, 'm');
legend('Fx_{front}','Fy_{front}','Fy_{rear}'); xlabel('t [s]'); grid on;

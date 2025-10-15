

function dx = vehicle_kinematic_model(t, x, u, L)
% VEHICLE_KINEMATIC_MODEL  Modèle cinématique du véhicule (bicycle model)
%
% Entrées :
%   t : temps (non utilisé ici, utile pour ode45)
%   x : [x ; y ; psi]  (états : position et cap)
%   u : [v ; delta]    (entrées : vitesse et angle de braquage)
%   L : empattement (m)
%
% Sortie :
%   dx : dérivées [dx ; dy ; dpsi]

    % Déballage des variables
    psi = x(3);
    v = u(1);
    delta = u(2);

    % Équations du modèle cinématique
    dx = zeros(3,1);
    dx(1) = v * cos(psi);        % dx/dt
    dx(2) = v * sin(psi);        % dy/dt
    dx(3) = (v / L) * tan(delta); % dpsi/dt
end

function x_next = vehicle_kinematic_step(x, u, L, dt)
% Intégration discrète du modèle cinématique
    psi = x(3);
    v = u(1);
    delta = u(2);

    x_next = zeros(3,1);
    x_next(1) = x(1) + dt * v * cos(psi);
    x_next(2) = x(2) + dt * v * sin(psi);
    x_next(3) = x(3) + dt * (v/L) * tan(delta);
end


dt = 0.1;
T = 10;
N = T/dt;

x = [0;0;0];
L = 2.5;
v = 5;
delta = 10*pi/180;
u = [v; delta];

traj = zeros(3,N);

for k = 1:N
    traj(:,k) = x;
    x = vehicle_kinematic_step(x, u, L, dt);
end

plot(traj(1,:), traj(2,:), 'LineWidth', 2);
xlabel('x [m]'); ylabel('y [m]');
title('Simulation discrète (Euler explicite)');
grid on; 
axis equal;
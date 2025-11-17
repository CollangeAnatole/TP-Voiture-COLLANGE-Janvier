function [t, X, Y, psi, r, delta_f] = modele_cinematique_4roues(ParaV, ParaS)
% modele_cinematique_4roues : modèle cinématique simple pour véhicule 4 roues
dt = ParaS.dt;
T = ParaS.Tf;
t = 0:dt:T;
N = length(t);

v = ParaS.v0; % m/s

% build beta time-series from scalar, vector or function handle
beta = zeros(size(t));
for k=1:N
    tk = t(k);
    if isa(ParaS.beta,'function_handle')
        beta(k) = ParaS.beta(tk);
    elseif numel(ParaS.beta)==1
        beta(k) = ParaS.beta;
    else
        beta(k) = ParaS.beta(k);
    end
end
beta = beta * pi/180; % deg -> rad

X = zeros(size(t)); Y = zeros(size(t)); psi = zeros(size(t));
r = zeros(size(t)); delta_f = zeros(size(t));

x = 0; y = 0; th = 0;
for k = 1:N
    psi_dot = (v / ParaV.L) * tan(beta(k));
    x = x + v * cos(th) * dt;
    y = y + v * sin(th) * dt;
    th = th + psi_dot * dt;

    X(k) = x; Y(k) = y; psi(k) = th; r(k) = psi_dot; delta_f(k) = beta(k);
end
end

function [t, U, V, r, X, Y, psi, Fx_front_total, Fy_front_total, Fy_rear_total] = model_dynamique_4roues(ParaV, ParaS, ParaT)
% model_dynamique_4roues : modèle dynamique 3-DOF pour véhicule 4 roues traction avant
dt = ParaS.dt;
T = ParaS.Tf;
t = (0:dt:T)';
N = length(t);
m = ParaV.m; Iz = ParaV.Iz; lf = ParaV.lf; lr = ParaV.lr; L = ParaV.L;
h = ParaV.h; Rw = ParaV.Rw; Ca_f = ParaV.Ca_f; Ca_r = ParaV.Ca_r;
mu = ParaV.mu; CdA = ParaV.CdA; Crr = ParaV.Crr;

U = zeros(N,1); V = zeros(N,1); r = zeros(N,1);
X = zeros(N,1); Y = zeros(N,1); psi = zeros(N,1);

U(1) = ParaS.v0; V(1)=0; r(1)=0; X(1)=0; Y(1)=0; psi(1)=0;

Fx_front_total = zeros(N,1);
Fy_front_total = zeros(N,1);
Fy_rear_total  = zeros(N,1);

g = 9.81;
Fz_f0 = ParaV.Fz_static_front; % total front
Fz_r0 = ParaV.Fz_static_rear;  % total rear

for k=1:N-1
    tk = t(k);
    Uk = U(k); Vk = V(k); rk = r(k);

    % braquage
    if isa(ParaS.beta,'function_handle')
        delta = ParaS.beta(tk) * pi/180;
    elseif numel(ParaS.beta)==1
        delta = ParaS.beta * pi/180;
    else
        delta = ParaS.beta(k) * pi/180;
    end

    % couple moteur
    if isa(ParaS.drive,'function_handle')
        Tdrive = ParaS.drive(tk);
    elseif numel(ParaS.drive)==1
        Tdrive = ParaS.drive;
    else
        Tdrive = ParaS.drive(k);
    end
    F_drive_brut = Tdrive / Rw; % force avant totale possible (sans limite)

    % résistances
    F_roll = Crr * m * g * sign(Uk + 1e-6);
    F_aero = CdA * Uk^2;
    ax_est = (F_drive_brut - F_roll - F_aero) / m;

    % transfert de charge longitudinal simple
    Fz_f = Fz_f0 - (m * ax_est * h) / L;
    Fz_r = Fz_r0 + (m * ax_est * h) / L;
    Fz_f = max(Fz_f, 1e-3); Fz_r = max(Fz_r, 1e-3);

    % adhérence front totale
    F_trac_max_front = mu * Fz_f;
    F_drive = sign(F_drive_brut) * min(abs(F_drive_brut), F_trac_max_front);
    Fx_front_total(k) = F_drive;

    % slip angles essieu
    if abs(Uk) < 0.1
        Uk_use = 0.1;
    else
        Uk_use = Uk;
    end
    alpha_f = atan2((Vk + lf * rk), Uk_use) - delta;
    alpha_r = atan2((Vk - lr * rk), Uk_use);

    % paramètres pneus
    if ~isempty(ParaT)
        Ca_f_eff = ParaT.Ca_f; Ca_r_eff = ParaT.Ca_r; Fymax = ParaT.Fymax;
    else
        Ca_f_eff = Ca_f; Ca_r_eff = Ca_r; Fymax = mu * (Fz_r0 + Fz_f0)/4;
    end

    Fy_f = - 2 * Ca_f_eff * alpha_f;
    Fy_r = - 2 * Ca_r_eff * alpha_r;

    % saturation non-linéaire
    Fy_f = Fymax * tanh(Fy_f / Fymax);
    Fy_r = Fymax * tanh(Fy_r / Fymax);

    Fy_front_total(k) = Fy_f;
    Fy_rear_total(k)  = Fy_r;

    sumFx = F_drive - F_roll - F_aero;
    sumFy = Fy_f + Fy_r;
    Mz = lf * Fy_f - lr * Fy_r;

    U_dot = rk * Vk + sumFx / m;
    V_dot = -rk * Uk + sumFy / m;
    r_dot = Mz / Iz;

    U(k+1) = Uk + U_dot * dt;
    V(k+1) = Vk + V_dot * dt;
    r(k+1) = rk + r_dot * dt;

    psi(k+1) = psi(k) + r(k+1) * dt;
    X(k+1) = X(k) + (U(k+1) * cos(psi(k)) - V(k+1) * sin(psi(k))) * dt;
    Y(k+1) = Y(k) + (U(k+1) * sin(psi(k)) + V(k+1) * cos(psi(k))) * dt;
end

% fill last entries
lastFx = find(Fx_front_total,1,'last');
if ~isempty(lastFx)
    Fx_front_total(end) = Fx_front_total(lastFx);
end
lastFyf = find(Fy_front_total,1,'last');
if ~isempty(lastFyf)
    Fy_front_total(end) = Fy_front_total(lastFyf);
end
lastFyr = find(Fy_rear_total,1,'last');
if ~isempty(lastFyr)
    Fy_rear_total(end) = Fy_rear_total(lastFyr);
end
end

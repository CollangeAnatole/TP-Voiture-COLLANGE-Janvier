function P = creerPneumatique_simple(Ca_front, Ca_rear, Fy_max_per_tire)
% creerPneumatique_simple : paramètres simplifiés de pneumatique
P.Ca_f = Ca_front; % N/rad per tire
P.Ca_r = Ca_rear;
P.Fymax = Fy_max_per_tire; % par pneu
end

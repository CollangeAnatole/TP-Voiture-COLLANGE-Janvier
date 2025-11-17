function V = creerVehicule4(m, Iz, lf, lr, h_cg, Rw, Ca_f_tire, Ca_r_tire, mu, CdA, Crr)
% creerVehicule4 : paramètres pour véhicule 4 roues (traction avant)
%   m : masse [kg]
%   Iz : inertie lacet [kg.m^2]
%   lf : distance CG -> essieu avant [m]
%   lr : distance CG -> essieu arrière [m]
%   h_cg : hauteur du centre de gravité [m]
%   Rw : rayon de roue [m]
%   Ca_f_tire : corner stiffness par pneu avant [N/rad]
%   Ca_r_tire : corner stiffness par pneu arrière [N/rad]
%   mu : coefficient d'adhérence global (estimation)
%   CdA : trainée aérodynamique (rho*Cd*A/2 factor) [kg/m]
%   Crr : coefficient de résistance au roulement

V.m = m;
V.Iz = Iz;
V.lf = lf;
V.lr = lr;
V.L = lf + lr;
V.h = h_cg;
V.Rw = Rw;
V.Ca_f = Ca_f_tire; % par pneu
V.Ca_r = Ca_r_tire; % par pneu
V.mu = mu;
V.CdA = CdA; % facteur aérodynamique
V.Crr = Crr;
g = 9.81;
V.Fz_static_front = m*g * lr / (lf+lr); % total sur essieu avant
V.Fz_static_rear  = m*g * lf / (lf+lr); % total sur essieu arrière
end

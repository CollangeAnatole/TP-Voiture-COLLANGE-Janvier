function S = creerSim(v_kmh, beta_deg_profile, Tf, dt, drive_profile)
% creerSim : paramètres de simulation
%   v_kmh : vitesse initiale / consigne en km/h (converti en m/s)
%   beta_deg_profile : fonction/valeur du braquage [deg] (peut être scalar, vector ou function handle)
%   Tf : durée [s]
%   dt : pas de simulation [s]
%   drive_profile : fonction/valeur de couple moteur [N.m] (total aux roues avant)
S.v0 = v_kmh / 3.6; % m/s
S.beta = beta_deg_profile;
S.Tf = Tf;
S.dt = dt;
S.drive = drive_profile;
end

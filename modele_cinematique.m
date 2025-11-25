function [t,U,Beta,delta_f,delta_r,r,r_ng,ay,X,Y] = modele_cinematique(Vehicule,Sim)

    % --- Paramètres véhicule ---
    Lf = Vehicule.Lf;
    Lr = Vehicule.Lr;
    L  = Lf + Lr;

    % --- Génération des scénarios depuis Sim ---
    [U,Beta,t] = creation_scenario(Sim.scenario_vitesse, ...
                                   Sim.scenario_braquage, ...
                                   Sim.vmax/3.6, ...   % conversion km/h -> m/s
                                   Sim.acceleration, ...
                                   Sim.Beta, ...
                                   Sim.Tf);

    dt = t(2)-t(1);

    % --- Conditions initiales ---
    x = 0; y = 0; psi = 0;

    % --- Préallocation ---
    X = zeros(size(t));
    Y = zeros(size(t));
    PSI = zeros(size(t));
    r = zeros(size(t));
    r_ng = zeros(size(t));
    ay = zeros(size(t));
    delta_f = zeros(size(t));
    delta_r = zeros(size(t));

    % --- Boucle de simulation (formule simple) ---
    for k = 1:length(t)
        v_k = U(k);
        delta_k = Beta(k);

        dx = v_k * cos(psi);
        dy = v_k * sin(psi);
        psi_dot = (v_k / L) * tan(delta_k);

        % Mise à jour
        x = x + dx * dt;
        y = y + dy * dt;
        psi = psi + psi_dot * dt;

        % Sorties
        r(k)      = psi_dot;
        r_ng(k)   = v_k / L * sin(delta_k);     % approx sans glissement
        ay(k)     = v_k * psi_dot;
        delta_f(k)= delta_k - atan((Lf * psi_dot) / max(v_k, eps));
        delta_r(k)= -atan((Lr * psi_dot) / max(v_k, eps));

        X(k)   = x;
        Y(k)   = y;
        PSI(k) = psi;
    end
end
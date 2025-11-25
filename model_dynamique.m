function [t,u,beta,delta,d_teta,teta,x,y] = model_dynamique(ParaV,ParaS)

    %% Constantes
    m  = ParaV.masse;
    Iz = ParaV.Iz;
    Lf = ParaV.Lf;
    Lr = ParaV.Lr;
    Cf = ParaV.Cf;
    Cr = ParaV.Cr;

    %% Temps
    dt = 1/100;
    temps_final = ParaS.Tf;
    t = (0:dt:temps_final-dt)';
    l = length(t);

    %% Entrées
    [u,beta,t] = creation_scenario(ParaS.scenario_vitesse, ...
                                   ParaS.scenario_braquage, ...
                                   ParaS.vmax/3.6, ...
                                   ParaS.acceleration, ...
                                   ParaS.Beta, ...
                                   ParaS.Tf);

    %% États
    teta   = zeros(l,1);
    d_teta = zeros(l,1);
    delta  = zeros(l,1);

    %% Boucle
    for i = 2:l
        u_i = max(u(i-1), 1.0); % vitesse minimale
        beta_i = beta(i-1);

        % Matrices A et B
        a11 = -(Lf^2*Cf + Lr^2*Cr)/(u_i*Iz);
        a12 = (-Lf*Cf + Lr*Cr)/Iz;
        a21 = (-Cf*Lf + Cr*Lr)/(m*u_i^2) - 1;
        a22 = -(Cf + Cr)/(m*u_i);

        b1 = (Lf*Cf)/Iz;
        b2 = Cf/(m*u_i);

        % Équations différentielles
        dd_teta = a11*d_teta(i-1) + a12*delta(i-1) + b1*beta_i;
        dd_delta = a21*d_teta(i-1) + a22*delta(i-1) + b2*beta_i;

        % Intégration
        teta(i)   = teta(i-1)   + d_teta(i-1)*dt;
        d_teta(i) = d_teta(i-1) + dd_teta*dt;
        delta(i)  = delta(i-1)  + dd_delta*dt;
    end

    %% Trajectoire
    x = zeros(l,1); y = zeros(l,1);
    for i = 2:l
        u_i = max(u(i-1), 1.0);
        psi = teta(i-1) + delta(i-1); % delta = angle de dérive CG
        x(i) = x(i-1) + u_i*cos(psi)*dt;
        y(i) = y(i-1) + u_i*sin(psi)*dt;
    end
end

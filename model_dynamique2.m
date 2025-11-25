%modèle utilisant Pacejka pour le contact sol roue

function [t,u,beta,delta,d_teta,teta,x,y] = model_dynamique2(ParaV,ParaS,pneu)

    %% Constantes véhicule
    m   = ParaV.masse;
    Iz  = ParaV.Iz;
    Lf  = ParaV.Lf;
    Lr  = ParaV.Lr;

    %% Pneus (forces par essieu)
    mu = pneu.mu;
    Fzf = 2 * pneu.Fz;  % 2 pneus à l'avant
    Fzr = 2 * pneu.Fz;  % 2 pneus à l'arrière

    Df = mu * Fzf;
    Dr = mu * Fzr;

    Bf = pneu.Bf; Cf = pneu.Cf; Ef = pneu.Ef;
    Br = pneu.Br; Cr = pneu.Cr; Er = pneu.Er;

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
        u_i = max(u(i-1), 1.0);
        beta_i = beta(i-1);

        v_i = delta(i-1) * u_i;

        alpha_f = atan((v_i + Lf*d_teta(i-1))/u_i) - beta_i;
        alpha_r = atan((v_i - Lr*d_teta(i-1))/u_i);

        Fy_f = Df * sin(Cf * atan(Bf*alpha_f - Ef*(Bf*alpha_f - atan(Bf*alpha_f))));
        Fy_r = Dr * sin(Cr * atan(Br*alpha_r - Er*(Br*alpha_r - atan(Br*alpha_r))));

        ddpsi = (Lf*Fy_f - Lr*Fy_r)/Iz;
        ddel  = (Fy_f + Fy_r)/(m*u_i) - d_teta(i-1);

        teta(i)   = teta(i-1)   + d_teta(i-1)*dt;
        d_teta(i) = d_teta(i-1) + ddpsi*dt;
        delta(i)  = delta(i-1)  + ddel*dt;
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

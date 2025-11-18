%modèle utilisant Pacejka pour le contact sol roue

function [t,u,beta,delta,d_teta,teta,x,y] = model_dynamique2(ParaV,ParaS,pneu)

    %% Constantes véhicule
    m   = ParaV.masse;        
    Iz  = ParaV.Iz;           
    Lf  = ParaV.Lf;           
    Lr  = ParaV.Lr;           
    g   = 9.81;               

    %% Paramètres sol et pneus (Pacejka)
    mu  = pneu.mu;            
    Fzf = pneu.Fz;           
    Fzr = pneu.Fz;           

    % Pacejka avant
    Bf = pneu.Bf; Cf = pneu.Cf; Ef = pneu.Ef; Df = mu*Fzf;
    % Pacejka arrière
    Br = pneu.Br; Cr = pneu.Cr; Er = pneu.Er; Dr = mu*Fzr;

    %% Temps
    dt           = 1/100;
    temps_final  = ParaS.Tf;
    t            = (0:dt:temps_final-dt)';
    l            = length(t);

    %% Entrées
    u0_kmh   = ParaS.v;       
    beta0_deg= ParaS.Beta;    
    u0       = u0_kmh/3.6;    
    beta0    = beta0_deg*pi/180;

    u     = u0 * ones(l,1);
    beta  = beta0 * ones(l,1);

    %% États
    teta      = zeros(l,1);   
    d_teta    = zeros(l,1);   
    delta     = zeros(l,1);   

    % Init
    teta(1)   = 0;
    d_teta(1) = 0;
    delta(1)  = 0;

    %% Boucle d'intégration
    for i = 2:l
        u_i     = u(i-1);
        beta_i  = beta(i-1);
        dpsi_i  = d_teta(i-1);
        delta_i = delta(i-1);

        % Vitesse latérale au CG
        v_i = delta_i * u_i;

        % Angles de dérive pneus
        alpha_f = atan((v_i + Lf*dpsi_i)/u_i) - beta_i;
        alpha_r = atan((v_i - Lr*dpsi_i)/u_i);

        % Forces latérales Pacejka
        Fy_f = Df * sin(Cf * atan(Bf*alpha_f - Ef*(Bf*alpha_f - atan(Bf*alpha_f))));
        Fy_r = Dr * sin(Cr * atan(Br*alpha_r - Er*(Br*alpha_r - atan(Br*alpha_r))));

        % Dynamiques
        ddpsi = (Lf*Fy_f - Lr*Fy_r) / Iz;
        ddel  = (Fy_f + Fy_r) / (m*u_i) - dpsi_i;

        % Intégration Euler
        teta(i)   = teta(i-1)   + dpsi_i*dt;
        d_teta(i) = d_teta(i-1) + ddpsi*dt;
        delta(i)  = delta(i-1)  + ddel*dt;
    end

    %% Trajectoire
    x  = zeros(l,1); y  = zeros(l,1);
    x(1) = 0; y(1) = 0;

    for i = 2:l
        u_i    = u(i-1);
        psi_i  = teta(i-1) + delta(i-1);
        x(i)   = x(i-1) + u_i*cos(psi_i)*dt;
        y(i)   = y(i-1) + u_i*sin(psi_i)*dt;
    end
end
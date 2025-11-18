function[t,u,beta,delta,d_teta,teta,x,y] = model_dynamique(ParaV,ParaS)
    
    %% Declaration des Constantes

    m=ParaV.masse; % Masse en Kg
    Iz=ParaV.Iz; % Moment d'inertie du lacet en Kg.m-2
    Lf=ParaV.Lf; % Demi empatement avant en m
    Lr=ParaV.Lr; % Demi empatement arrière en m
    Cf=ParaV.Cf; % Rigidité de dérive avant en N.rad-1
    Cr=ParaV.Cr; % Rigidité de dérive arrière en N.rad-1
    
    
    %% Vecteur temps
    
    dt=1/100; % delta temps en s
    temps_final=ParaS.Tf;
    
    t=(0:dt:temps_final-dt)';
    l=temps_final/dt;
    
    
    %% Entrées du système
    
    [u,beta,t] = creation_scenario(ParaS.scenario_vitesse, ...
                           ParaS.scenario_braquage, ...
                           ParaS.vmax/3.6, ...   % conversion km/h → m/s
                           ParaS.acceleration, ...
                           ParaS.Beta, ...
                           ParaS.Tf);

    
    %% Variables
    
    teta=zeros(l,1);
    d_teta=zeros(l,1);
    delta=zeros(l,1);
    
    
    %% Initialisation des variable
    
    % Initialisation variables
    teta_1=0;
    d_teta_1=0;
    delta_1=0;
    
    teta(1)=teta_1;
    d_teta(1)=d_teta_1;
    delta(1)=delta_1;
    
    
    %% Resolution equa diff
    
    for i=2 : l
    
        % Calcul des coeeficient des matrice A et B
    
        % Calcul matriciel du style derivé de X = A X + B u
        % On a ici X dui est le vecteur colonne composé de la derivé de teta et de
        % delta respectivement teta angle de lacet et delta angle de derive au
        % centre de gravité en rad
    
        u_1=u(i-1);
        beta_1=beta(i-1);
    
        % Calcul des coeeficient matrice A
    
        a11_1=-(Lf^2*Cf+Lr^2*Cr)/(u_1*Iz);
        a12_1=(-Lf*Cf+Lr*Cr)/(Iz);
        a21_1=(-Cf*Lf+Cr*Lr)/(m*u_1^2)-1;
        a22_1=-(Cf+Cr)/(m*u_1);
        
        % Création des coeeficient de la matrice B
    
        b1_1=(Lf*Cf)/Iz;
        b2_1=Cf/(m*u_1);

    
    
        % Dérivées
    
        d_d_teta_1=a11_1*d_teta_1+a12_1*delta_1+b1_1*beta_1;
        d_delta_1=a21_1*d_teta_1+a22_1*delta_1+b2_1*beta_1;
        
        
        % Resolution de l'equation differentiel
        
        teta_2=teta_1+d_teta_1*dt;
        d_teta_2=d_teta_1+d_d_teta_1*dt;
        delta_2=delta_1+d_delta_1*dt;
    
    
        % Inscription dans les vecteurs
    
        teta(i)=teta_2;
        d_teta(i)=d_teta_2;
        delta(i)=delta_2;
    
    
        % Reinitalise
    
        teta_1=teta_2;
        d_teta_1=d_teta_2;
        delta_1=delta_2;
        
       
    end

    %% Tracé de la trajectoire

    x_1=0;
    y_1=0;
    
    x=zeros(l,1);
    y=zeros(l,1);
    
    x(1)=x_1;
    y(1)=y_1;
    
    for i = 2 : l
        
        u_1=u(i-1);
        teta_1=teta(i-1);
        delta_1=delta(i-1);
        psi=teta_1+delta_1;
    
        x_2=x_1+u_1*cos(psi)*dt;
        y_2=y_1+u_1*sin(psi)*dt;
    
        x(i)=x_2;
        y(i)=y_2;
    
        x_1=x_2;
        y_1=y_2;
        
    end

end
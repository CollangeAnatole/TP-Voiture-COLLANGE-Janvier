clc


%% Declaration des Constantes

m=1310; % Masse en Kg
Iz=1760; % Moment d'inertie du lacet en Kg.m-2
Lf=1.2; % Demi empatement avant en m
Lr=1.4; % Demi empatement arrière en m
Cf=69740; % Rigidité de dérive avant en N.rad-1
Cr=63460; % Rigidité de dérive arrière en N.rad-1

%% Vecteur temps

dt=1/100; % delta temps en s
temps_final=20;

t=(0:dt:temps_final-dt)';
l=temps_final/dt;


%% Entrées du système

u_0=2; % Vitesse lineaire du centre de gravité du vehicule en m.s-1 constant
u=u_0*ones(l,1);

beta_0=2; % Angle de braquage en rad
beta=beta_0*ones(l,1);


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


%% Boucle for (corp du programe)

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
    
    
    % Resolution de l'equation differentiel
    
    teta_2=
    d_teta_2=
    delta_2=
    
    
    
    
end



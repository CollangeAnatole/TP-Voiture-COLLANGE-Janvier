function[t,u,beta,delta,d_teta,teta,x,y] = model_dynamique2(ParaV,ParaS,paraP)
    
   m  = ParaV.masse;   % masse (kg)
Iz = ParaV.Iz;      % moment d'inertie (kg.m2)
Lf = ParaV.Lf;      % distance CG-avant (m)
Lr = ParaV.Lr;      % distance CG-arrière (m)

%% Temps
dt = 1/100;                   % pas de temps (s)
t = (0:dt:ParaS.Tf-dt)';      % vecteur temps
l = length(t);

%% Entrées
u_0   = ParaS.v / 3.6;        % vitesse en m/s
beta_0 = ParaS.Beta * pi/180; % angle de dérive initial en rad

u    = u_0*ones(l,1);
beta = beta_0*ones(l,1);

%% Initialisation variables
teta   = zeros(l,1);
d_teta = zeros(l,1);
delta  = zeros(l,1);

teta(1)   = 0;
d_teta(1) = 0;
delta(1)  = 0;

%% Boucle dynamique (Euler)
for i = 2:l
    % Valeurs précédentes
    teta_1   = teta(i-1);
    d_teta_1 = d_teta(i-1);
    delta_1  = delta(i-1);
    u_1      = u(i-1);
    
    % --- Forces pneus Dugoff ---
    [Fx_front, Fy_front] = modele_pneumatique(paraP, delta_1);
    [Fx_rear,  Fy_rear ] = modele_pneumatique(paraP, delta_1);
    
    % --- Dynamique du lacet ---
    d_d_teta_1 = (Lf*Fy_front - Lr*Fy_rear) / Iz;
    
    % --- Variation de l'angle de dérive / delta ---
    d_delta_1  = (Fx_front + Fx_rear) / (m*u_1);
    
    % --- Intégration Euler ---
    teta_2   = teta_1 + d_teta_1*dt;
    d_teta_2 = d_teta_1 + d_d_teta_1*dt;
    delta_2  = delta_1 + d_delta_1*dt;
    
    % --- Stockage ---
    teta(i)   = teta_2;
    d_teta(i) = d_teta_2;
    delta(i)  = delta_2;
end

%% Trajectoire (Euler)
x = zeros(l,1);
y = zeros(l,1);

x(1) = 0;
y(1) = 0;

for i = 2:l
    psi = teta(i-1) + delta(i-1); % angle approximatif du CG
    x(i) = x(i-1) + u(i-1) * cos(psi) * dt;
    y(i) = y(i-1) + u(i-1) * sin(psi) * dt;
end

end
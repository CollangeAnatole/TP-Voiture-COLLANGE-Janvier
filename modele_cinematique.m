function [t,u,beta,delta_f,delta_r,r,r_ng,ay,X,Y]=modele_cinematique(ParaV,ParaS)
    %---------------------------------------------------------------
    % SIMULATEUR CINÉMATIQUE D'UN VÉHICULE (modèle bicycle)
    % Sorties : angles de dérive géométriques, vitesses de lacet,
    %           trajectoire, vitesse et accélération latérale du CdG
    %---------------------------------------------------------------

    % --- Paramètres du véhicule ---
 
    Lf = ParaV.Lf;      % Demi-empattement avant [m]
    Lr = ParaV.Lr;      % Demi-empattement arrière [m]
    L = Lf + Lr;   % Empattement total [m]

    % --- Paramètres de simulation ---
    dt = 0.01;     % Pas de temps [s]
    T  = ParaS.Tf;       % Durée [s]
    t  = 0:dt:T;

    % --- Entrées ---
    v = ParaS.v;                                      % Vitesse constante [m/s]
    beta = ParaS.Beta*pi/180 * ones(size(t));         % Braquage constant 
    u_0=v/3.6; % Vitesse lineaire du centre de gravité du vehicule en m.s-1 constant
    u=u_0*ones(size(t));

    % --- Conditions initiales ---
    x = 0; y = 0; psi = 0;

    % --- Préallocation ---
    X = zeros(size(t));
    Y = zeros(size(t));
    PSI = zeros(size(t));
    r = zeros(size(t));         % vitesse de lacet
    r_ng = zeros(size(t));      % vitesse de lacet "sans glissement"
    ay = zeros(size(t));        % accélération latérale
    delta_f = zeros(size(t));   % dérive avant
    delta_r = zeros(size(t));   % dérive arrière

    % --- Boucle de simulation ---
    for k = 1:length(t)
        % Cinématique du véhicule
        dx = v * cos(psi);
        dy = v * sin(psi);
        psi_dot = (v / L) * tan(beta(k));

        % Mises à jour
        x = x + dx * dt;
        y = y + dy * dt;
        psi = psi + psi_dot * dt;

        % Sorties géométriques
        r(k) = psi_dot;               % vitesse de lacet réelle
        r_ng(k) = v / L * sin(beta(k)); % vitesse de lacet sans glissement (approx)
        ay(k) = v * psi_dot;          % accélération latérale du CdG
        delta_f(k) = beta(k) - atan(Lf * psi_dot / v);
        delta_r(k) = -atan(Lr * psi_dot / v);

        % Stockage position/orientation
        X(k) = x;
        Y(k) = y;
        PSI(k) = psi;
    end
% 
%     % --- Affichages ---
%     figure;
%     plot(X, Y, 'b', 'LineWidth', 2);
%     xlabel('x [m]'); ylabel('y [m]');
%     title('Trajectoire du véhicule (modèle cinématique)');
%     axis equal; grid on;
% 
%     figure;
%     subplot(3,1,1);
%     plot(t, r*180/pi, 'r', 'LineWidth', 1.5);
%     hold on;
%     plot(t, r_ng*180/pi, 'k--', 'LineWidth', 1);
%     ylabel('Vitesse de lacet [°/s]');
%     legend('Réelle','Sans glissement'); grid on;
% 
%     subplot(3,1,2);
%     plot(t, delta_f*180/pi, 'b', t, delta_r*180/pi, 'm', 'LineWidth', 1.5);
%     ylabel('Angles de dérive [°]');
%     legend('\alpha_f','\alpha_r'); grid on;
% 
%     subplot(3,1,3);
%     plot(t, ay, 'c', 'LineWidth', 1.5);
%     xlabel('Temps [s]');
%     ylabel('a_y [m/s²]');
%     grid on;
% end
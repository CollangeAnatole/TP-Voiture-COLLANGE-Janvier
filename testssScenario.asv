% Script de test des scénarios
clear; close all; clc;

% Paramètres communs
v_max = 30;       % vitesse max en m/s
a_max = 2;        % accélération max en m/s^2
b = 5;            % braquage initial en degrés
Tf = 20;          % durée simulation
dt = 0.01;        % pas de temps
t = (0:dt:Tf-dt)';

% Liste des scénarios
scenariosVitesse  = {'constant','acceleration','freinage'};
scenariosBraquage = {'constant','virage','zigzag'};

figure;
plotIndex = 1;

for sv = 1:length(scenariosVitesse)
    for sb = 1:length(scenariosBraquage)
        % Génération du scénario
        [U,Beta,t] = creation_scenario(scenariosVitesse{sv}, ...
                                       scenariosBraquage{sb}, ...
                                       v_max,a_max,b,Tf);

        % Dérivées numériques
        dU    = gradient(U,t);
        dBeta = gradient(Beta,t);

        % Affichage vitesse et dérivée
        subplot(length(scenariosVitesse),length(scenariosBraquage),plotIndex);
        plot(t,U,'b','LineWidth',1.5); hold on;
        plot(t,dU,'r--','LineWidth',1.2);
        xlabel('Temps (s)'); ylabel('Vitesse / dérivée');
        title(['Vitesse: ',scenariosVitesse{sv},' | Braquage: ',scenariosBraquage{sb}]);
        legend('U','dU/dt');

        plotIndex = plotIndex + 1;
    end
end

figure;
plotIndex = 1;

for sv = 1:length(scenariosVitesse)
    for sb = 1:length(scenariosBraquage)
        % Génération du scénario
        [U,Beta,t] = creation_scenario(scenariosVitesse{sv}, ...
                                       scenariosBraquage{sb}, ...
                                       v_max,a_max,b,Tf);

        % Dérivées numériques
        dU    = gradient(U,t);
        dBeta = gradient(Beta,t);

        % Affichage braquage et dérivée
        subplot(length(scenariosVitesse),length(scenariosBraquage),plotIndex);
        plot(t,Beta,'b','LineWidth',1.5); hold on;
        plot(t,dBeta,'r--','LineWidth',1.2);
        xlabel('Temps (s)'); ylabel('Braquage / dérivée');
        title(['Vitesse: ',scenariosVitesse{sv},' | Braquage: ',scenariosBraquage{sb}]);
        legend('Beta','dBeta/dt');

        plotIndex = plotIndex + 1;
    end
end

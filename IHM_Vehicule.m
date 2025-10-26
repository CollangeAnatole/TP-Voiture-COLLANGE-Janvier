function IHM_Dynamique
    % Interface MATLAB R2024b - Simulation modèle dynamique véhicule

    % --- Fenêtre principale ---
    fig = uifigure('Name','Simulation Dynamique Véhicule','Position',[100 100 1000 600]);

    %% --- Panneau paramètres ---
    pnl = uipanel(fig,'Title','Paramètres','FontWeight','bold','Position',[20 50 350 500]);

    % Véhicule
    uilabel(pnl,'Text','Masse (kg):','Position',[10 400 100 22]);
    massField = uieditfield(pnl,'numeric','Position',[150 400 100 22],'Value',1310);

    uilabel(pnl,'Text','Iz (kg.m²):','Position',[10 370 100 22]);
    IzField = uieditfield(pnl,'numeric','Position',[150 370 100 22],'Value',1760);

    uilabel(pnl,'Text','Lf (m):','Position',[10 340 100 22]);
    LfField = uieditfield(pnl,'numeric','Position',[150 340 100 22],'Value',1.2);

    uilabel(pnl,'Text','Lr (m):','Position',[10 310 100 22]);
    LrField = uieditfield(pnl,'numeric','Position',[150 310 100 22],'Value',1.4);

    uilabel(pnl,'Text','Cf:','Position',[10 280 100 22]);
    CfField = uieditfield(pnl,'numeric','Position',[150 280 100 22],'Value',69740);

    uilabel(pnl,'Text','Cr:','Position',[10 250 100 22]);
    CrField = uieditfield(pnl,'numeric','Position',[150 250 100 22],'Value',63460);

    % Simulation
    uilabel(pnl,'Text','Vitesse (km/h):','Position',[10 200 150 22]);
    vField = uieditfield(pnl,'numeric','Position',[150 200 100 22],'Value',250);

    uilabel(pnl,'Text','Angle Beta (°):','Position',[10 170 150 22]);
    BetaField = uieditfield(pnl,'numeric','Position',[150 170 100 22],'Value',5);

    uilabel(pnl,'Text','Durée (s):','Position',[10 140 100 22]);
    TfField = uieditfield(pnl,'numeric','Position',[150 140 100 22],'Value',20);

    % Bouton lancer simulation
    uibutton(pnl,'Text','Lancer Simulation','FontWeight','bold',...
        'BackgroundColor',[0.4 0.8 0.4],'Position',[70 80 200 40],...
        'ButtonPushedFcn',@(btn,event) lancerSimulation());

    %% --- Axes résultats ---
    % Vitesse
    ax1 = uiaxes(fig,'Position',[400 380 550 200]);
    title(ax1,'Vitesse du véhicule'); xlabel(ax1,'Temps [s]'); ylabel(ax1,'Vitesse [km/h]'); grid(ax1,'on');

    % Trajectoire
    ax2 = uiaxes(fig,'Position',[400 100 550 200]);
    title(ax2,'Trajectoire du véhicule'); xlabel(ax2,'x [m]'); ylabel(ax2,'y [m]'); grid(ax2,'on'); axis(ax2,'equal');

    % Angles β, δ, θ, dθ/dt
    ax3 = uiaxes(fig,'Position',[50 20 850 70]);
    title(ax3,'Angles β, δ, θ, dθ/dt'); xlabel(ax3,'Temps [s]'); ylabel(ax3,'Angle [°]'); grid(ax3,'on');

    %% --- Fonction interne : lancer la simulation ---
    function lancerSimulation()
        try
            % Créer structures
            ParaV = creerVehicule(massField.Value,IzField.Value,LfField.Value,LrField.Value,CfField.Value,CrField.Value);
            ParaS = creerSim(vField.Value,BetaField.Value,TfField.Value);

            % Appel du modèle dynamique
            [t,u,beta,delta,d_teta,teta,x,y] = model_dynamique(ParaV,ParaS);

            % --- Tracé vitesse ---
            plot(ax1,t,u*3.6,'b','LineWidth',1.5);
            title(ax1,'Vitesse du véhicule'); xlabel(ax1,'Temps [s]'); ylabel(ax1,'Vitesse [km/h]'); grid(ax1,'on');

            % --- Tracé trajectoire ---
            plot(ax2,x,y,'r','LineWidth',1.5);
            title(ax2,'Trajectoire du véhicule'); xlabel(ax2,'x [m]'); ylabel(ax2,'y [m]'); grid(ax2,'on'); axis(ax2,'equal');

            % --- Tracé angles ---
            plot(ax3,t,beta*180/pi,'b',t,delta*180/pi,'r',t,teta*180/pi,'k',t,d_teta,'m--','LineWidth',1.2);
            legend(ax3,{'β','δ','θ','dθ/dt'});
            title(ax3,'Angles β, δ, θ, dθ/dt'); xlabel(ax3,'Temps [s]'); ylabel(ax3,'Angle [°]'); grid(ax3,'on');

        catch ME
            % Afficher une alerte en cas d'erreur
            uialert(fig,ME.message,'Erreur Simulation');
        end
    end
end

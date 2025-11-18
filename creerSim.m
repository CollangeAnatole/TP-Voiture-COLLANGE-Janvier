%mise en place des paramètres de similation pour les modèles dynamiques
%cinématiques
function sim= creerSim(v,Beta,acceleration,scenario_vitesse,scenario_braquage)
    sim.vmax=v;
    sim.Beta=Beta;
    sim.Tf=20;
    sim.scenario_vitesse=scenario_vitesse;
    sim.scenario_braquage=scenario_braquage;
    sim.acceleration=acceleration;
end
%mise en place des paramètres deu véhicule pour le modèle dynamique

function pvehi=creerVehicule(masse,Iz,Lf,Lr,Cf,Cr)
    pvehi.masse = masse;
    pvehi.Iz = Iz;
    pvehi.Lf = Lf;
    pvehi.Lr = Lr;
    pvehi.Cf = Cf;
    pvehi.Cr = Cr;
end
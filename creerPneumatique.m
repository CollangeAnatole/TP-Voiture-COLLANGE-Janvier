function pneu = creerPneumatique(Fz,mu,Ky,Kx,gl,Bf,Cf,Ef,Br,Cr,Er)
    % Paramètres généraux (utiles pour modèle linéaire)
    pneu.Fz = Fz;      % charge verticale [N]
    pneu.mu = mu;      % coefficient de friction
    pneu.Ky = Ky;      % raideur latérale (modèle linéaire)
    pneu.Kx = Kx;      % raideur longitudinale (modèle linéaire)
    pneu.gl = gl;      % taux de glissement longitudinal

    % Paramètres Pacejka (modèle non linéaire)
    pneu.Bf = Bf;       % stiffness factor (avant)
    pneu.Cf = Cf;       % shape factor (avant)
    pneu.Ef = Ef;       % curvature factor (avant)

    % Pour simplifier, on peut utiliser les mêmes pour l'arrière
    pneu.Br = Br;       % stiffness factor (arrière)
    pneu.Cr = Cr;       % shape factor (arrière)
    pneu.Er = Er;       % curvature factor (arrière)
end

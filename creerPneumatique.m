function pneu = creerPneumatique(Fz, mu, Ky, Kx, gl, B, C, E)
    % Paramètres généraux (utiles pour modèle linéaire)
    pneu.Fz = Fz;      % charge verticale [N]
    pneu.mu = mu;      % coefficient de friction
    pneu.Ky = Ky;      % raideur latérale (modèle linéaire)
    pneu.Kx = Kx;      % raideur longitudinale (modèle linéaire)
    pneu.gl = gl;      % taux de glissement longitudinal

    % Paramètres Pacejka (modèle non linéaire)
    pneu.Bf = B;       % stiffness factor (avant)
    pneu.Cf = C;       % shape factor (avant)
    pneu.Ef = E;       % curvature factor (avant)

    % Pour simplifier, on peut utiliser les mêmes pour l'arrière
    pneu.Br = B;       % stiffness factor (arrière)
    pneu.Cr = C;       % shape factor (arrière)
    pneu.Er = E;       % curvature factor (arrière)
end

%mise en place des parametres des pneumatiques

function pneu=creerPneumatique(gl,Fz,mu,Kx,Ky)
    pneu.gl=gl;%taux de glissement longitudinal
    pneu.Fz = Fz;%charge vertcale
    pneu.mu = mu;%coef de frottement fixe
    pneu.Kx = Kx;%raideur longitudinale
    pneu.Ky = Ky;%raideur laterale

end
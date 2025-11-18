function [U,Beta,t] = creation_scenario(scenarioVitesse,scenarioBraquage,v_max,a_max,b,Tf)
    if nargin < 6, Tf = 20; end
    dt = 0.01;
    t = (0:dt:Tf-dt)';
    l = length(t);

    % Phase constante au début (20%)
    l_const = round(0.2*l);
    t_const = t(l_const);

    U0 = 5; Beta0 = b*pi/180;

    % Fonction de transition lissée (cosstep)
    cosstep = @(tau) 0.5*(1-cos(pi*tau));
    tau = (t-t_const)/(Tf-t_const);
    tau = max(0,min(1,tau));

    % Vitesse
    switch scenarioVitesse
        case 'constant'
            U = U0*ones(l,1);
        case 'acceleration'
            U = U0 + (v_max-U0)*cosstep(tau);
        case 'freinage'
            U = U0 - (U0-0.1)*cosstep(tau);
    end

    % Braquage
    switch scenarioBraquage
        case 'constant'
            Beta = Beta0*ones(l,1);
        case 'virage'
            Beta = Beta0 + ((b+15)*pi/180 - Beta0)*cosstep(tau);
        case 'zigzag'
            Beta = Beta0 + 5*pi/180*sin(2*pi*0.2*t).*cosstep(tau);
    end
end

clc

V1=creerVehicule(1310,1760,1.2,1.4,69740,63460);
S1=creerSim(250,5);
S1=creerSim(50,5,20);
pneu1 = creerPneumatique(4000, 0.9, 80000, 60000, 0.01, 10, 1.9, 0.97);

[t,u,beta,delta,d_teta,teta,x,y]=model_dynamique2(V1,S1,pneu1);
[tps,v,Beta,delta_f,delta_r,r,r_ng,ay,X,Y]=modele_cinematique(V1,S1);
%% Affichage

figure(1)
subplot(5,1,1)
plot(t, u*3.6);
title('Vitesse du vehicule en fonction du temps');
xlabel("s")
ylabel("Km.h-1")
grid on;

subplot(5,1,2)
plot(t, beta*180/pi);
title('Angle de braquage en fonction du temps');
xlabel("s")
ylabel("deg")
grid on;

subplot(5,1,3)
plot(t, delta*180/pi);
title('Angle de derive en foction du temps');
xlabel("s")
ylabel("deg")
grid on;

subplot(5,1,4)

plot(t, teta*180/pi);
title('\theta');
xlabel("s")
ylabel("deg")

grid on;

subplot(5,1,5)
plot(t,d_teta);
title("Dérivé de l'angele de lacet en foction du temps");
xlabel("s")
ylabel("deg.s-1")
grid on;

figure(2);

plot(x,y);
title("Trajectoire du vehicule");
xlabel("m");
ylabel("m");
axis equal;

%affichage cinématique
    figure(3);
    plot(X, Y, 'b', 'LineWidth', 2);
    xlabel('x [m]'); ylabel('y [m]');
    title('Trajectoire du véhicule (modèle cinématique)');
    axis equal; grid on;

    figure(4);
    subplot(3,1,1);
    plot(tps, r*180/pi, 'r', 'LineWidth', 1.5);
    hold on;
    plot(tps, r_ng*180/pi, 'k--', 'LineWidth', 1);
    ylabel('Vitesse de lacet [°/s]');
    legend('Réelle','Sans glissement'); grid on;

    subplot(3,1,2);
    plot(tps, delta_f*180/pi, 'b', tps, delta_r*180/pi, 'm', 'LineWidth', 1.5);
    ylabel('Angles de dérive [°]');
    legend('\alpha_f','\alpha_r'); grid on;

    subplot(3,1,3);
    plot(tps, ay, 'c', 'LineWidth', 1.5);
    xlabel('Temps [s]');
    ylabel('a_y [m/s²]');
    grid on;
clc

V1=creerVehicule(1310,1760,1.2,1.4,69740,63460);
S1=creerSim(250,5,20);

[t,u,beta,delta,d_teta,teta,x,y]=model_dynamique(V1,S1);

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

figure(2)

plot(x,y)
title("Trajectoire du vehicule")
xlabel("m")
ylabel("m")
axis equal
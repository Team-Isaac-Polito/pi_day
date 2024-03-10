clc; clear all, close all

% lunghezza dei due link del braccio
l1 = 5;
l2 = 5;

% posizione finale dell'end-effector
x_end = 0;
y_end = 9;

%traiettoria lineare tra la posizone di default e la posizione finale
%i punti intermedi sono equamente distanziati
x = linspace(0.1,x_end,50);
y = linspace(0,y_end,50);

% formula del coseno per trovare theta2
c2 = (x.^2+y.^2-l1^2-l2^2)/(2*l1*l2);
s2 = sqrt(1-c2.^2);
% tangente inversa per trovare l'angolo theta2
theta2 = atan2d(s2,c2);
% equazione per trovare l'angolo theta1
theta1 = atan2d(y,x)-atan2d(l2*s2,l1+l2*c2);

% simulazione del braccio planare
pausa=0.05;
sim_planar_arm(theta1,theta2,l1,l2,pausa)

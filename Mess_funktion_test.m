%% Multidimensional Kalman-Filter
% DSV2 Projekt
clear all; close all; clc;


d=100;

% x=-5;
% y=30;
% pi-acos(y/(d-x))
% tan(y/(d-x))


figure(1)
x=6;
y=30;
xt=@(x)tand(y/(d-x));
fplot(xt,'Linewidth',2)

hold on 
for j=-80:10:80

y=j;


xt=@(x)tan(y/(d-x));
fplot(xt,[-pi pi],'Linewidth',2)

load_klasse= ['d-wert' int2str(d) '   y-wert' int2str(y)];
legende{j+81,:}=load_klasse;

end
legend(legende);
title('beta');
set(gca,'FontSize',20)

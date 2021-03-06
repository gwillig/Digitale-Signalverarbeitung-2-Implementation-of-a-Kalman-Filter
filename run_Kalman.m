%% Multidimensional Kalman-Filter
% DSV2 Projekt
clear all; close all; clc;
%% Messungen f�r die Roboterposition generieren
s.dt =1;%Zeitschritt

szene=2;
switch szene
    case 1 %Geradeausfahrt
x=-1:1:60;
m=2;b=60;
y=m*x+b;
%y=15*sin(x);
noise_gain=0.0008; %um diesen Wert schwanken, Rauschen, Rausch-Gain
    case 2 %Sinus abfahren
x=-100:4:100;
m=15;b=50;
y=m*sin(x*pi/32)+b;

w_noise=0.5;
noise_gain=w_noise*(pi/180); %um diesen Wert schwanken, Rauschen, Rausch-Gain
    case 3 % Kreisfahrt
t=0:2*pi/60:2*pi;
r=20;x_0=0;y_0=50;
y=r*sin(t)+y_0;
x=r*cos(t)+x_0;

w_noise=0.5;
noise_gain=w_noise*(pi/180); %um diesen Wert schwanken, Rauschen, Rausch-Gain       
end

it=length(x); %Anzahl der Messungen
s.d=120;%Kameradistanz

beta  = atan2(y,(s.d-x));
gamma = atan2(y,(s.d+x)); 
% beta  = pi-(acos(y./(s.d-x)));
% gamma = acos(y./(s.d+x)); 

% x' y'
figure();
Messung = [beta+noise_gain*randn(1,it);gamma+noise_gain*randn(1,it)];
Messung_GPS=[x+noise_gain;y+noise_gain];
figure('Name','Generierte Messdaten')
plot(1:it,beta);hold on;plot(1:it,gamma);plot(1:it,Messung);legend('beta','gamma','beta-Messung','gamma-Messung');
%% Initiale Startposition:
%     [      x;     y; dx; dy; dxR;alpha ;dalpha];
s.x = [ x(1);y(1);  0;  0;   0;     0;     0];
s.P = 10*eye(7); % Initiale Abweichung der Startposition

% Process Noise Covariance f�r die Pr�diktion (Konstante)
% s.Q=0.01*eye(7);%Achtung!--> schwachsinnig f�r alle Statevektor-Daten die
% gleiche Varianz anzunehmen
%Besser:
winkelfehler=30;
Gewicht_Q=1;
s.Q=Gewicht_Q*[1,0,0,0,0,0,0;...
       0,1,0,0,0,0,0;...
       0,0,2,0,0,0,0;...
       0,0,0,2,0,0,0;...
       0,0,0,0,2,0,0;...
       0,0,0,0,0,winkelfehler*(pi/180),0;...
       0,0,0,0,0,0,winkelfehler*(pi/180)];

%% Messungen durchgehen
out=[];
out_nach_pred=[];
varianz_x=[];
varianz_y=[];
varianz_x_nachp=[];
varianz_y_nachp=[];
for k=1:it
out=horzcat(out,s.x);%Kompletten Statevektor f�r jedenzeitschritt k in out schreiben
varianz_x=horzcat(varianz_x,s.P(1,1));%Varianz f�r jeden Zeitschritt mitschreiben
varianz_y=horzcat(varianz_y,s.P(2,2));

%% Zeitupdate, predict (Position sch�tzen)
s=predict(s);

out_nach_pred=horzcat(out_nach_pred,s.x);

s.P
%% Messupdate (korregieren)
%Messung aufrufen:
Z=Messung(:,k);
winkelrauschen=15;
R_Matrix_Winkelsensor=winkelrauschen*(pi/180)*eye(2);
%s=messupdate(s,Z,R_Matrix_Winkelsensor);
%Z_GPS=Messung_GPS(:,k);
%R_GPS=0.1*eye(2);
%s=messupdate_GPS(s,Z_GPS,R_GPS);



varianz_x_nachp=horzcat(varianz_x_nachp,s.P(1,1));%Varianz f�r jeden Zeitschritt mitschreiben
varianz_y_nachp=horzcat(varianz_y_nachp,s.P(2,2));

%Verrauschte Messdaten auf XY-zur�ckrechnen (nur zur Darstellung)
x_berechnet(k)=s.d*(tan(Z(1))-tan(Z(2)))/(tan(Z(1))+tan(Z(2)));
y_berechnet(k)=2*s.d*tan(Z(1))*tan(Z(2))/(tan(Z(1))+tan(Z(2)));
% x_berechnet(k)=s.d*(atan(Z(1))-atan(Z(2)))/(atan(Z(1))+atan(Z(2)));
% y_berechnet(k)=2*s.d*atan(Z(1))*atan(Z(2))/(atan(Z(1))+atan(Z(2)));
% x_berechnet(k)=  s.d*(cos(Z(1))+cos(Z(2)))/(cos(Z(1))-cos(Z(2)));
% y_berechnet(k)=2*s.d*(cos(Z(1))*cos(Z(2)))/(cos(Z(1))-cos(Z(2)));

end

%% Plot der Daten
%% X-Y-Positionen
figure('Name','Roboterposition in Weltkoordinaten')
plot(-s.d,0,'LineStyle','none','Marker','O','MarkerSize',10,'MarkerFaceColor',[0.85 0.33 0.1],'MarkerEdgeColor','k','LineWidth',1)
hold on
plot(s.d,0,'LineStyle','none','Marker','O','MarkerSize',10,'MarkerFaceColor',[0 0.400000005960464 0],'MarkerEdgeColor','k','LineWidth',1)
plot(x,y,'LineWidth',1.5,'color',[0 0.447058826684952 0.74117648601532])
plot(out_nach_pred(1,:),out_nach_pred(2,:),'--','LineWidth',1.5,'color',[0.635294139385223 0.0784313753247261 0.184313729405403])
plot(out(1,:),out(2,:),'LineWidth',1.5,'color',[0.929411768913269 0.694117665290833 0.125490203499794])

labels = cellstr( num2str([1:it]') );  %' # labels correspond to their order
plot(x_berechnet,y_berechnet,'LineStyle','none','Marker','*','color',[0.466666668653488 0.674509823322296 0.18823529779911])
text(x_berechnet,y_berechnet, labels, 'VerticalAlignment','bottom', ...
                             'HorizontalAlignment','right')

legend('Winkelsensor beta','Winkelsensor gamma','ist Position','Pr�diktion','Messupdate','Messdaten');set(gca,'FontSize',20)
axis([-s.d s.d 0  max(y)])
;xlabel('Position X');ylabel('Position Y');

%% Varianz
figure('Name','Varianz der Position')
plot(varianz_x,'LineWidth',1.5);hold on;xlabel('Diskrete Zeitschritte');ylabel('Varianz in m');set(gca,'FontSize',20)
plot(varianz_y,'LineWidth',1.5)

plot(varianz_x_nachp,'LineWidth',1.5)
plot(varianz_y_nachp,'LineWidth',1.5)

%Differenz zur Wahrenposition nach Messupdate:
plot(out(1,:)-x,'LineWidth',1.5)
plot(out(2,:)-y,'LineWidth',1.5)
%Differenz zur Wahrenposition nach Pr�diktion:
plot(out_nach_pred(1,:)-x,'LineWidth',1.5)
plot(out_nach_pred(2,:)-y,'LineWidth',1.5)

legend('Varianz in x','Varianz in y','Varianz in x nach pr�d','Varianz in y nach pr�d','Differenz nach messupdate von X','Differenz nach messupdate von Y','Differenz nach pr�diktion von X','Differenz nach pr�diktion von Y');

% errorbar(out(1,:),out(2,:),varianz_y,varianz_y,varianz_x,varianz_x,'o')

figure('Nam','Geschwindigkeit')
plot(out(3,:)); hold on;plot(out(6,:));plot(out(5,:));hold off;xlabel('Diskrete Zeitschritte');ylabel('Geschwindigkeit in m/sec');legend('Geschwindigkeit in X','Geschwindigkeit in Y','Geschwindigkeit in Roboterrichtung');set(gca,'FontSize',20);

figure('Name','winkel')
plot(out(5,:));xlabel('Diskrete Zeitschritte');ylabel('Winkel in Rad');set(gca,'FontSize',20);

figure('Name','Winkelbeschleunigung')
plot(out(6,:));xlabel('Diskrete Zeitschritte');ylabel('Winkel in Rad/sec');set(gca,'FontSize',20);


set_monitor_figure(2,2,3)

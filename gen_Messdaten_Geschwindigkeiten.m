function [dphi_links,dphi_rechts] = gen_Messdaten_Geschwindigkeiten(x,y,delta_t,alpha_start)

%% Input-Parameter
% x:= x-Soll
% y:= y-Soll
% s.dt:= delta t vom struct s
% d:= Radabstand vom Roboterkoordinatensydtem zum Radmittelpunkt
% r:= Radradius

%% Output-Parameter
% dphi_links:= Winkelgeschwindigkeit des linken Rades
% dphi_rechts:= Winkelgeschwindigkeit des rechten Rades

% % % % % % % % % Initialwerte zum Zeitpunkt t=0
% % % % % % % % % Man könnte auch die Initialwerte aus dem Statevektor ablesen
% % % % % % % % % x_start = -2.5;     % x-Position zum Zeitpunkt t=0
% % % % % % % % % y_start = 0;        % y-Position zum Zeitpunkt t=0
% % % % % % % % % t_start = 0;        % Zeitpunkt "t=0"
% % % % % % % % % Teilungsfaktor = 4;
% % % % % % % % % alpha_start = (pi/4); % Ausrichtung des Roboters zur x-Achse des Weltkoordinatensystems zum Zeitpunkt t=0
% % % % % % % % % alpha_start = pi+pi/4
% % % % % % % % % alpha_start_winkel = (alpha_start*180)/pi


d = 2;              % Radabstand
r = 0.5;            % Radradius

% % % % % % % % % %% Definition des zu fahrenden Pfades
% % % % % % % % % 
% % % % % % % % % szene = 1;
% % % % % % % % % switch (szene)
% % % % % % % % %     case 1
% % % % % % % % %         % Pfad in Form eines Kreises mit Radius (R) und Kreismittelpunkt Pm(xm/ym)
% % % % % % % % %         R = 5;
% % % % % % % % %         xm = 3; ym = 3;
% % % % % % % % %         winkel = pi:-2*pi/Teilungsfaktor:0;
% % % % % % % % %         x = R*cos(winkel) + xm
% % % % % % % % %         y = R*sin(winkel) + ym
% % % % % % % % %     case 2
% % % % % % % % %         % Pfad in Form einer Geraden (IST OK)
% % % % % % % % % %         alpha_start = pi/4
% % % % % % % % %         x = [10:-1:0]
% % % % % % % % %         y = 1*x+10    
% % % % % % % % %   case 3
% % % % % % % % %         % Pfad in Form eines Sinus
% % % % % % % % % %         alpha_start = atan(1/(pi/2))
% % % % % % % % %         T = 2*pi;
% % % % % % % % %         x = [0:T/4:T]
% % % % % % % % %         y = 1*sin((2*pi/T)*x)
% % % % % % % % % end
% % % % % % % % % Anz_Streckenzuege = length(x)-1;
% % % % % % % % % plot(x,y)
% % % % % % % % % xlim([min(x)-1,max(x)+1])
% % % % % % % % % ylim([min(y)-1,max(y)+1])

% Streckendifferenz (x_(t) - x_(t-1)) und (y_(t) - y_(t-1))
delta_x = diff(x);
delta_y = diff(y);

dx = delta_x/delta_t; % Geschwindigkeit in x-Weltkoordinaten
dy = delta_y/delta_t; % Geschwindigkeit in y-Weltkoordinaten

% Berechnung der Geschwindigkeit des Roboter in x-Roboterkoordinatn
dxR = sqrt(dx.^2+dy.^2);

% Resultat ist Winkel in rad von den jeweiligen Streckenabschnitten
alpha = [alpha_start,atan2(delta_y,delta_x)];
% alpha_winkel = (alpha*180)/pi

% % % % % % % % % % % % for i=1:length(alpha)
% % % % % % % % % % % %  if alpha(i) < 0
% % % % % % % % % % % %     Vorzeichen(i) = 1; 
% % % % % % % % % % % %  elseif alpha(i) > 0
% % % % % % % % % % % %     Vorzeichen(i) = 0;
% % % % % % % % % % % %  end
% % % % % % % % % % % % end
% % % % % % % % % % % % 
% % % % % % % % % % % % diff_Vorzeichen = diff(Vorzeichen)
% % % % % % % % % % % % 
% % % % % % % % % % % % for i=1:length(diff_Vorzeichen)
% % % % % % % % % % % %    if diff_Vorzeichen(i) == 1
% % % % % % % % % % % %    alpha(i+1) = 2*pi + alpha(i+1);
% % % % % % % % % % % %    end
% % % % % % % % % % % % end
% % % % % % % % % % 
% % % % % % % % % % % alpha_winkel = (alpha*180)/pi

% Winkeldifferenz (a_(t) - a_(t-1)) für die Winkelrate dalpha 
delta_alpha = diff(alpha);
% delta_alpha_winkel = (delta_alpha*180)/pi
% delta_winkel = (dalpha*180)/pi;
dalpha = delta_alpha/delta_t;
% dalpha_winkel = (dalpha*180)/pi

dphi_links= dxR/r - (d/r)*dalpha;
dphi_rechts= dxR/r + (d/r)*dalpha;
end 


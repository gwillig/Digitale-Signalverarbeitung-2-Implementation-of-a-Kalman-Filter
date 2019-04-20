function s=messupdate(s,Z,R_Matrix)
% Correction für aktuellen Zeitschritt bzw. Messung z_k
% Correction

% measurement noise covariance Konstante

%Messrauschen alles rein was "böse" ist (bitte nicht in die Kovarianz packen!)
R=R_Matrix;

% phi_1=-s.x(2)/((s.d+s.x(1))^2);
% phi_2= s.x(2)/((s.d-s.x(1))^2);
% H=[ phi_2  , 1/(s.d-s.x(1)) ,0,0,0,0,0;...
%     phi_1  , 1/(s.d+s.x(1)) ,0,0,0,0,0];


phi_1=(s.x(2)^2/((s.d-s.x(1))^2))+1;
phi_2=(s.x(2)^2/((s.d+s.x(1))^2))+1;
H=[ s.x(2)/(phi_1*(s.d-s.x(1))^2)  , 1/(phi_1*(s.d-s.x(1))) ,0,0,0,0,0;...
   -s.x(2)/(phi_2*(s.d+s.x(1))^2)  , 1/(phi_2*(s.d+s.x(1))) ,0,0,0,0,0];


% phi_1=atan2((s.x(2)/(s.d-s.x(1) ))^2)+1;
% phi_2=atan2((s.x(2)/(s.d+s.x(1) ))^2)+1;
% H=[ s.x(2)*phi_1/(s.d-s.x(1))^2  , phi_1/(s.d-s.x(1)) ,0,0,0,0,0;...
%    -s.x(2)*phi_2/(s.d+s.x(1))^2  , phi_2/(s.d+s.x(1)) ,0,0,0,0,0];
% Statevektor in Messgröße beta und gamma abbilden:
h_x(1)= atan2(s.x(2),( s.d-s.x(1) ));
h_x(2)= atan2(s.x(2),( s.d+s.x(1) ));

% phi_1=sqrt(1-(s.x(2)^2/(s.d-s.x(1))^2));
% phi_2=sqrt(1-(s.x(2)^2/(s.d+s.x(1))^2));
% H=[s.x(2)/((s.d-s.x(1))^2*phi_1) , 1/((s.d-s.x(1))*phi_1) ,0,0,0,0,0;...
%    s.x(2)/((s.d+s.x(1))^2*phi_2),-1/((s.d+s.x(1))*phi_2),0,0,0,0,0];
% Statevektor in Messgröße beta und gamma abbilden:
% h_x(1)= pi - (acos(s.x(2)/(s.d-s.x(1))));%beta
% h_x(2)=acos(s.x(2)/(s.d+s.x(1))); %gamma


% Messvektor in xy Größe des Statevektors abbilden:
% z_x(1)= s.d*(cos(Z(1))+cos(Z(2)))/(cos(Z(1))-cos(Z(2)));    %x-Wert
% z_x(2)=2*s.d*(cos(Z(1)).*cos(Z(2)))./(cos(Z(1))-cos(Z(2))); %y-Wert
% y=z_x'-[s.x(1);s.x(2)]; % Differenz aus Messwerten und prädizierten Messwerten des Statevektors

y=Z-h_x';%winkeldifferenz aus Messwerten und prädizierten Messwerten des Statevektors
S=(H*s.P*H'+R); 
K=s.P*H'*inv(S); % Filter-Matrix (Kalman-Gain)
%Vertraue zu 100% dem Messwert!!
%K=ones(7,2)*10;

s.x=s.x+(K*y); % aktualisieren des Statevektors

I = eye(7); % Identity matrix
s.P=(I-(K*H))*s.P; % aktualisieren der Kovarianz

end
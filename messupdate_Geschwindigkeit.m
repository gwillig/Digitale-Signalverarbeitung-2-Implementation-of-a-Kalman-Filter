function s = messupdate_Geschwindigkeit(s,Z)
%% Normaler K-Filter

r=0.5;
d=1;
% measurement noise covariance
R = 0.5*eye(2);  %Messrauschen alles rein was "böse" ist (nicht in die Kovarianz packen!)
%Linearer Kalmanfilter:
%y=Z-(H*s.x); % Differenz aus Messwerten und prädizierten Messwerten

%==== H-Matrix

% % H_x(1)= ((s.x(5)-Radabstand)/R);%phi_links`
% % H_x(2)=((s.x(5)+Radabstand)/R); %phi_rechts

       %x   y   dx  dy   dxR    alpha   dalpha
H = [   0   0   0   0    1/r    0       -d/r;...   %d_phi_links (Winkelgeschwindigkeit vom linken Rad)
        0   0   0   0    1/r    0       d/r];      %d_phi_rechts (Winkelgeschwindigkeit vom rechts Rad)

    
y=Z-H*s.x; % Differenz aus Messwerten und prädizierten Messwerten des Statevektors
S=(H*s.P*H'+R); % InnovationskovarianzK=P*H'*inv(S); % Filter-Matrix (Kalman-Gain)
K=s.P*H'*inv(S); % Filter-Matrix (Kalman-Gain)
s.x=s.x+(K*y); % aktualisieren des Statevektors
I = eye(7); % Identity matrix
s.P=(I-(K*H))*s.P; % aktualisieren d er Kovarianz


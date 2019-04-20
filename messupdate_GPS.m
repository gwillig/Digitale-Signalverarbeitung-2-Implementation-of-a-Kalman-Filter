function s=messupdate_test(s,Z,R_GPS)
% s
% z für alktuellen Zeitschritt
% Correction

H=[ 1 0 0 0 0 0 0;0 1 0 0 0 0 0];

R = R_GPS;

y=Z-(H*s.x); % Innovation aus Messwertdifferenz
S=(H*s.P*H'+R); % InnovationskovarianzK=P*H'*inv(S); % Filter-Matrix (Kalman-Gain)
K=s.P*H'*inv(S); % Filter-Matrix (Kalman-Gain)
s.x=s.x+(K*y); % aktualisieren des Systemzustand
I=eye(7);
s.P=(I-(K*H))*s.P; % aktualisieren der Kovarianz


end
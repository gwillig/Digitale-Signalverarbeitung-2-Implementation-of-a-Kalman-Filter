function s=predict(s)

%% Zustandsübergangsmatrix -->Bewegungsgleichungen, Dynamikmodell F oder A
%beschreibt den Systemübergang von t nach t-1 ohne Aktionen und Rauschen
% Transition Matrix

%Rücksubstitution des Statevektors
x       =s.x(1);
y       =s.x(2);
dx      =s.x(3);
dy      =s.x(4);
dxr     =s.x(5);
alpha	=s.x(6);
dalpha	=s.x(7);


f(5)=dxr;
f(6)=alpha+(dalpha*s.dt)/2;
f(7)=dalpha;

phi_1=sin(f(6));%bei pi/2 = 1
phi_2=cos(f(6));%bei pi/2 = 0
f(1)=dxr*s.dt*phi_2+x;  %Position in X
f(2)=dxr*s.dt*phi_1+y;  %Position in Y
f(3)=dxr*phi_2;         %Geschwindigkeit in x
f(4)=dxr*phi_1;         %Geschwindigkeit in y


%% Jacobi-Matrix
%      x, y,dx,dy,    dxr      ,    alpha          ,       dalpha
% s.F = [1, 0, 0, 0, s.dt*phi_2  , -dxr*s.dt*phi_1 , -dxr*s.dt^2*phi_1 ; ...
%        0, 1, 0, 0, s.dt*phi_1  , dxr*s.dt*phi_2 , dxr*s.dt^2*phi_2 ; ...
%        0, 0, 0, 0, phi_2       , -dxr*phi_1      , -dxr*s.dt*phi_1   ; ...
%        0, 0, 0, 0, phi_1       , dxr*phi_2      , dxr*s.dt*phi_2   ; ...
%        0, 0, 0, 0, 1, 0, 0     ; ...
%        0, 0, 0, 0, 0, 1, s.dt  ; ...
%        0, 0, 0, 0, 0, 0, 1    ];
s.F = [1, 0, 0, 0, s.dt*phi_2  , -dxr*s.dt*phi_1 , -(dxr*s.dt^2*phi_1) ; ...
       0, 1, 0, 0, s.dt*phi_1  , dxr*s.dt*phi_2 , (dxr*s.dt^2*phi_2) ; ...
       0, 0, 0, 0, phi_2       , -dxr*phi_1      , -(dxr*s.dt*phi_1)   ; ...
       0, 0, 0, 0, phi_1       , dxr*phi_2      , (dxr*s.dt*phi_2)   ; ...
       0, 0, 0, 0, 1, 0, 0     ; ...
       0, 0, 0, 0, 0, 1, s.dt  ; ...
       0, 0, 0, 0, 0, 0, 1    ];
 
  phi_1=sin(f(6));%bei pi/2 = 1
phi_2=cos(f(6));%bei pi/2 = 0 

%Linearer Kalmanfilter:
%s.x=s.F*s.x; % Prädizierter Zustand aus Bisherigem und System
s.x=f';
%Kovarianzmatrix gibt ein Maß für die Güte der Schätzung des Zustandsvektors
s.P=(s.F*s.P*s.F')+s.Q; % Kovarianz updaten Prädizieren der Kovarianz


end
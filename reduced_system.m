%% Initial Param
f = 50;
h = 1/f;
gamma = 0.5;
gh = gamma^h;

%% Reference Model
c1=5;c2=3;c3=3*0.1571/3;c4=5;
yss=2;

%%%%%% x-DOF %%%%%%%%
ymtilde=((c1-c2)*cos(c3*ts)+c4*cos(((c1-c2)*c3/c2)*ts) + yss);

S=collect(laplace(ymtilde));
[nS,dS]=numden(S);
num=eval(coeffs(nS,'All'));
den=eval(coeffs(dS,'All'));
[Amc,Bmc,Cmc,Dmc]=tf2ss(num,den);

% [Vm,Em]=eig(Amc)
% invVm=inv(Vm);

Amx=expm(Amc*h);
Cmx=Cmc;
xm0x=Bmc;

%% 1DOF Linear Plant
Ac=[0 1;0 -1.5];Bc=[0 1.3]';Cc=[1 0];Dc=0;

% Continuos
sysc=ss(Ac,Bc,Cc,Dc);

% Discrete ZOH
sysZOH=c2d(sysc,h,'zoh');
Ad=sysZOH.a;Bd=sysZOH.b;Cd=sysZOH.c;

%% Initial gain %%%%%%%%%%%%%

%% A-dlyap
% A11 = [a11, a12; a21, a22];    % (Unknown plant)
A11 = Ad;   
A12 = zeros(2,5);  
A13 = Bd;           %%
A21 = zeros(5, 2);
A22 = Amx;    % Am (Complete knowledge)
A23 = zeros(5,1);    
A31 = -Kx(1:2)*A11;   %% Unkown
A32 = -Kx(3:7)*A22;
A33 = -Kx(1:2)*Bd; 


C = [Cd -Cmx];
Qe = 1;
R = 1;
Q_LQR = C'*Qe*C;
Q = [Q_LQR, zeros(7,1); zeros(1, 7), R];


Q11 = Q(1:2, 1:2);
Q12 = Q(1:2, 3:7);
Q13 = Q(1:2, 8);
Q21 = Q(3:7, 1:2);
Q22 = Q(3:7, 3:7);
Q23 = Q(3:7, 8);
Q31 = Q(8, 1:2);
Q32 = Q(8, 3:7);
Q33 = Q(8, 8);

Q = [Q11, Q12, Q13; Q21, Q22, Q23; Q31, Q32, Q33];



%% Subsystem A_kron*x = b_kron

% For the four equations
vecQ22 = Q22(:);
vecQ23 = Q23(:);
vecQ32 = Q32(:);
vecQ33 = Q33(:);

b_kron = [vecQ22; vecQ23; vecQ32; vecQ33];
b_kron_hardest = vecQ22;


%%EQ 5 
kron22_22 = kron(A22', A22');
kron32_22 = kron(A32', A22');
kron22_32 = kron(A22', A32');
kron32_32 = kron(A32', A32');

%%EQ 6 
kron22_13 = kron(A22', A13');
kron32_13 = kron(A32', A13');
kron22_33 = kron(A22', A33');
kron32_33 = kron(A32', A33');

%%EQ 8 
kron13_22 = kron(A13', A22');
kron33_22 = kron(A33', A22');
kron13_32 = kron(A13', A32');
kron33_32 = kron(A33', A32');

%%EQ 9 
kron13_13 = kron(A13', A13');
kron33_13 = kron(A33', A13');
kron13_33 = kron(A13', A33');
kron33_33 = kron(A33', A33');
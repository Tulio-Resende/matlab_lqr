


close all
clear all

%% Sampling time
f = 50;
h = 1/f;


%% 1DOF Linear Plant
Ac=[0 1;0 -1.5];Bc=[0 1.3]';Cc=[1 0];Dc=0;

% Continuos
sysc=ss(Ac,Bc,Cc,Dc);

% Discrete ZOH
sysZOH=c2d(sysc,h,'zoh');
Ad=sysZOH.a;Bd=sysZOH.b;Cd=sysZOH.c;

np=max(size(Ac));

%% Reference Generator Model
syms ts real

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

%%%%%% Y-DOF %%%%%%%%
ymtilde_y=((c1-c2)*sin(c3*ts)-c4*sin(((c1-c2)*c3/c2)*ts) + yss);

S=collect(laplace(ymtilde_y));
[nS,dS]=numden(S);
num=eval(coeffs(nS,'All'));
den=eval(coeffs(dS,'All'));
[Amc,Bmc,Cmc,Dmc]=tf2ss(num,den);

% [Vm,Em]=eig(Amc)
% invVm=inv(Vm);

Amy=expm(Amc*h);
Cmy=Cmc;
xm0y=Bmc;


nm=max(size(Amc));


%% Augumented System: Just Plant and Generator


Az=[Ad zeros(np,nm);
    zeros(nm,np) Amx];
Bz=[Bd;zeros(nm,1)];

%%%%% x-DOF %%%%%%%%
Czx=[Cd -Cmx];
%%%%% y-DOF %%%%%%%%
Czy=[Cd -Cmy];

% %% Augumented System: Plant, Integrator
% 
% Aa=[Ad zeros(np,1);
%     -h*Cd 1];
% Ba=[Bd; 0];
% 
% %%%%% x-DOF %%%%%%%%
% Cax=[Cd -1/h -Cmx];
% %%%%% y-DOF %%%%%%%%
% Cay=[Cd -1/h -Cmy];




%% Augumented System: Plant, Integrator and Generator

% Aa=[Ad zeros(np,1) zeros(np,nm);
%     -h*Cd 1 zeros(1, nm);
%     zeros(nm,np) zeros(nm, 1) Amx];
% Ba=[Bd; 0; zeros(nm,1)];
% 
%%%%% x-DOF %%%%%%%%
Cax=[Cd -1/h -Cmx];
%%%%% y-DOF %%%%%%%%
Cay=[Cd -1/h -Cmy];


%% Parameters LQR
% ----------------------------------------------------------------
R = 1;
Q = 1;
Qe = 1;

%% Parameters LQT
%%%%% x-DOF %%%%%%%%
Qzx=Czx'*Qe*Czx;
%%%%% y-DOF %%%%%%%%
Qzy=Czy'*Qe*Czy;


%% Parameters LQT-I
%%%%% x-DOF %%%%%%%%
Qax=Cax'*Qe*Cax;
%%%%% y-DOF %%%%%%%%
Qay=Cay'*Qe*Cay;


%%%%% Discout %%%%%
g = 0.5;
gh=g^h;
%% LQR

% [K] = dlqr(sqrt(gh)*Ad,sqrt(gh)*Bd,Q, R)

%% LQT

%%%% x-DOF %%%%%%%%
[Kx] = dlqr(sqrt(gh)*Az,sqrt(gh)*Bz,Qzx, R)

%%%%% y-DOF %%%%%%%%
[Ky] = dlqr(sqrt(gh)*Az,sqrt(gh)*Bz,Qzy, R)


%% LQT-I
g = 0.5;
gh=g^h;
% 
% %%%%% x-DOF %%%%%%%%
% [Kx] = dlqr(sqrt(gh)*Aa,sqrt(gh)*Ba,Qax, R)
% 
% %%%%% y-DOF %%%%%%%%
% [Ky] = dlqr(sqrt(gh)*Aa,sqrt(gh)*Ba,Qay, R)




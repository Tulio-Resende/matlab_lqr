
syms ts real


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

%% Partial Knowledge of A-plant Matrix
syms kp;
Ac=[0 1;0 -kp];Bc=[0 1.3]';Cc=[1 0];Dc=0;
Ad = [1, (exp(kp/50) - 1)/kp; 0, exp(kp/50)];
np=max(size(Ac));

%% H-Matrix
% Symb H
syms h11 h12 h13 h14 h15 h16 h17 h18
syms h22 h23 h24 h25 h26 h27 h28
syms h33 h34 h35 h36 h37 h38
syms h44 h45 h46 h47 h48
syms h55 h56 h57 h58
syms h66 h67 h68
syms h77 h78
syms h88

% Construir matriz H simétrica
H = [h11 h12 h13 h14 h15 h16 h17 h18;
     h12 h22 h23 h24 h25 h26 h27 h28;
     h13 h23 h33 h34 h35 h36 h37 h38;
     h14 h24 h34 h44 h45 h46 h47 h48;
     h15 h25 h35 h45 h55 h56 h57 h58;
     h16 h26 h36 h46 h56 h66 h67 h68;
     h17 h27 h37 h47 h57 h67 h77 h78;
     h18 h28 h38 h48 h58 h68 h78 h88];

% Ground Truth for the Kstar

% H = [52.6334101099504,	20.6024519330030,	-469.319000517058,	4.97721748485912,	-11.3226277100745,	0.0906870518258517,	-0.0284979569785798,	0.530219715912279;
% 20.6024519330030,	10.2162277797894,	-182.594783493921,	2.79074753761095,	-4.41177716547765,	0.0508711785706812,	-0.0111550398808185,	0.264235786975587;
% -469.319000517058,	-182.594783493921,	4188.82853290319,	-43.1367941100211,	101.035079891101,  -0.785613683152772,	0.254109180044117,	-4.69850502927987;
% 4.97721748485912,	2.79074753761095,	-43.1367941100211,	1.03540441873538,	-1.04796182410773,	0.0189372165527760,	-0.00269487630499801,	0.0723516786020272;
% -11.3226277100745	-4.41177716547765	101.035079891101	-1.04796182410773	2.43711211624183	-0.0190877082447776	0.00613055009531251	-0.113527484683127;
% 0.0906870518258517,	0.0508711785706812,	-0.785613683152772,	0.0189372165527760,	-0.0190877082447776,	0.000346393806324802,	-4.91018099729528e-05,	0.00131887409816076;
% -0.0284979569785798,	-0.0111550398808185,	0.254109180044117,	-0.00269487630499801,	0.00613055009531251,	-4.91018099729528e-05,	1.54300006451501e-05,	-0.000287083406180558;
% 0.530219715912279,	0.264235786975587,	-4.69850502927987,	0.0723516786020272,	-0.113527484683127,	0.00131887409816076,	-0.000287083406180558,	1.00683497461433];

% Particionar H em blocos
H11 = H(1:2, 1:2);
H12 = H(1:2, 3:7);
H13 = H(1:2, 8);
H21 = H(3:7, 1:2);
H22 = H(3:7, 3:7);
H23 = H(3:7, 8);
H31 = H(8, 1:2);
H32 = H(8, 3:7);
H33 = H(8, 8);

% syms H11 H12 H13 H21 H22 H23 H31 H32 H33

H = [H11, H12, H13; H21, H22, H23; H31, H32, H33];
%% Theta Star 
test = [52.6334101099504 10.2162277797894 4188.82853290319 1.03540441873538 2.43711211624183 0.000346393806324802 1.54300006451501e-05 1.00683497461433 41.2049038660059 -938.638001034116 9.95443496971824 -22.6452554201491 0.181374103651703 -0.0569959139571596 1.06043943182456 -365.189566987842 5.58149507522191 -8.82355433095529 0.101742357141362 -0.0223100797616370 0.528471573951173 -86.2735882200421 202.070159782202 -1.57122736630554 0.508218360088234 -9.39701005855973 -2.09592364821546 0.0378744331055520 -0.00538975260999602 0.144703357204055 -0.0381754164895553 0.0122611001906250 -0.227054969366253 -9.82036199459057e-05 0.00263774819632151 -0.000574166812361116];
% Jacoud Version
% theta = [52.6334101099504; 10.2162277797894; 4188.82853290319; 1.03540441873538; 2.43711211624183; 0.000346393806324802; 1.54300006451501e-05; 1.00683497461433; 41.2049038660059; -938.638001034116; 9.95443496971824; -22.6452554201491; 0.181374103651703; -0.0569959139571596; 1.06043943182456; -365.189566987842; 5.58149507522191; -8.82355433095529; 0.101742357141362; -0.0223100797616370; 0.528471573951173; -86.2735882200421; 202.070159782202; -1.57122736630554; 0.508218360088234; -9.39701005855973; -2.09592364821546; 0.0378744331055520; -0.00538975260999602; 0.144703357204055; -0.0381754164895553; 0.0122611001906250; -0.227054969366253; -9.82036199459057e-05; 0.00263774819632151; -0.000574166812361116];
theta = test';
% H = F.FromTHETAtoP(theta, 8)
rank(H)

P_theta = zeros(36,36);
for i = 1:8
    P_theta(i,i) = 1;          % Elementos 1-8: mantém igual
end
for i = 9:36
    P_theta(i,i) = 0.5;        % Elementos 9-36: divide por 2
end

% New Version
theta = P_theta * theta;  

% H = F.FromTHETAtoP(theta, 8)

%% Gain Star
Kx = [0.526620279669378,	0.262442002550444,	-4.66660887607784,	0.0718605138143464,	-0.112756794852737,	0.00130992082259127,	-0.000285134518981841];

%% Setting up the A-dlyap Matrix
syms a11 a12 a21 a22

% Block A
% A11 = [a11, a12; a21, a22];    % (Unknown plant)
A11 = Ad;   
A12 = zeros(2,5);  
A13 = Bd;           %%
A21 = zeros(5, 2);
A22 = Amx;    % Am (Complete knowledge)
A23 = zeros(5,1);    
A31 = -Kx(1:2)*A11;   %% Unkown
A32 = -Kx(3:7)*A22;
A33 = -Kx(1:2)*Bd;      %%

% syms A11 A12 A13 A21 A22 A23 A31 A32 A33

A = [A11, A12, A13; A21, A22, A23; A31, A32, A33];


%% Setting up Q_dlyap matriz
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

% syms Q11 Q12 Q13 Q21 Q22 Q23 Q31 Q32 Q33
Q = [Q11, Q12, Q13; Q21, Q22, Q23; Q31, Q32, Q33];
% H=dlyap(A'*sqrt(gh),Q);

%% All the equations
J = gh*A'*H*A - H + Q;

%% Subsystem A_kron*x = b_kron

% For the four equations
vecQ22 = Q22(:);
vecQ23 = Q23(:);
vecQ32 = Q32(:);
vecQ33 = Q33(:);

b_kron = [vecQ22; vecQ23; vecQ32; vecQ33];
b_kron_hardest = vecQ22;
% b_kron_new = [vec22; vec33];
% b_kron_new = [vec32; vec33];
b_kron_new = [vecQ22; vecQ32];


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

function P = commutation_matrix(m, n)
    % Gera a matriz de comutação P_{m,n}
    % que satisfaz vec(X') = P * vec(X)
    P = zeros(m*n, m*n);
    for i = 1:m
        for j = 1:n
            row = (j-1)*m + i;  % posição em vec(X')
            col = (i-1)*n + j;  % posição em vec(X)
            P(row, col) = 1;
        end
    end
end


m = size(H12,1);
n = size(H21,1);

P_kron = commutation_matrix(m,n);

vec11 = H11(:);
vec21 = H21(:);
vec31 = H31(:);
vec12 = H12(:);
vec22 = H22(:);
vec32 = H32(:);
vec13 = H13(:);
vec23 = H23(:);
vec33 = H33(:);

% Easiest
% vecH = [vec11; vec21; vec31; vec12; vec22; vec32; vec13; vec23; vec33];
% vecH = [vec11; vec21; vec31; vec22; vec32; vec23; vec33];
vecH = [vec11; vec21; vec31; vec22; vec23; vec33];


% vecH_hardest = [vec22; vec32; vec23; vec33];
vecH_hardest = [vec22; vec23; vec33];


% New
%vecH_new = [vec11; vec31; vec22; vec23; vec33];
%vecH_new = [vec11; vec21; vec31; vec23; vec33];
vecH_new = [vec21; vec31; vec22; vec23; vec33];



% Easiest

% A_kron = [zeros(length(vec22), length(vec11) + length(vec21) + length(vec31) + length(vec12)), -gh*kron22_22 + eye(length(kron22_22)), -gh*kron22_32, zeros(length(vec22), length(vec13)), -gh*kron32_22 -gh*kron32_32;
%           zeros(5, length(vec11) + length(vec21) + length(vec31)), -gh*kron22_13, zeros(5, length(vec22)), -gh*kron22_33 + eye(length(vec32)), -gh*kron32_13, zeros(5,length(vec23)), -gh*kron32_33;
%           zeros(5, length(vec11)), -gh*kron13_22, -gh*kron13_32, zeros(5, length(vec12) + length(vec22) + length(vec32) + length(vec13)), -gh*kron33_22 + eye(length(vec23)) -gh*kron33_32;
%           -gh*kron13_13, zeros(1, length(vec21)), -gh*kron13_33, zeros(1, length(vec12) + length(vec22) + length(vec32)), -gh*kron33_13, zeros(1, length(vec23)), -gh*kron33_33 + eye(length(vec33))]

% A_kron = [zeros(length(vec22), length(vec11) + length(vec21) + length(vec31)), -gh*kron22_22 + eye(length(kron22_22)), -gh*kron22_32, -gh*kron32_22 -gh*kron32_32;
%           zeros(5, length(vec11)), -gh*kron22_13, -gh*kron32_13   zeros(5, length(vec22)), -gh*kron22_33 + eye(length(vec32)), zeros(5,length(vec23)), -gh*kron32_33;
%           zeros(5, length(vec11)), -gh*kron13_22, -gh*kron13_32, zeros(5, length(vec22) + length(vec32)), -gh*kron33_22 + eye(length(vec23)) -gh*kron33_32;
%           -gh*kron13_13, zeros(1, length(vec21)), -gh*(kron13_33 + kron33_13), zeros(1, length(vec22) + length(vec32) + length(vec23)), -gh*kron33_33 + eye(length(vec33))];


A_kron = [zeros(length(vec22), length(vec11) + length(vec21) + length(vec31)), -gh*kron22_22 + eye(length(kron22_22)), -gh*(kron22_32 + kron32_22) -gh*kron32_32;
          zeros(5, length(vec11)), -gh*kron22_13*P_kron, -gh*kron32_13   zeros(5, length(vec22)), -gh*kron22_33 + eye(length(vec32)), -gh*kron32_33;
          zeros(5, length(vec11)), -gh*kron13_22, -gh*kron13_32, zeros(5, length(vec22)), -gh*kron33_22 + eye(length(vec23)) -gh*kron33_32;
          -gh*kron13_13, zeros(1, length(vec21)), -gh*(kron13_33 + kron33_13), zeros(1, length(vec22) + length(vec23)), -gh*kron33_33 + eye(length(vec33))];

%A_kron_new = [zeros(length(vec22), length(vec11) + length(vec31)), -gh*kron22_22 + eye(length(kron22_22)), -gh*(kron22_32 + kron32_22) -gh*kron32_32;
%          -gh*kron13_13, -gh*(kron13_33 + kron33_13), zeros(1, length(vec22) + length(vec23)), -gh*kron33_33 + eye(length(vec33))];

% A_kron_new = [zeros(5, length(vec11)), -gh*kron13_22, -gh*kron13_32, -gh*kron33_22 + eye(length(vec23)) -gh*kron33_32;
%           -gh*kron13_13, zeros(1, length(vec21)), -gh*(kron13_33 + kron33_13), zeros(1, length(vec23)), -gh*kron33_33 + eye(length(vec33))];

A_kron_new = [zeros(25, length(vec21) + length(vec31)), -gh*kron22_22 + eye(length(kron22_22)), -gh*(kron22_32 + kron32_22) -gh*kron32_32;
              -gh*kron13_22, -gh*kron13_32, zeros(5, length(vec22)), -gh*kron33_22 + eye(length(vec23)) -gh*kron33_32];


% Hardest
% A_kron_hardest = [-gh*kron22_22 + eye(length(kron22_22)), -gh*kron22_32, -gh*kron32_22 -gh*kron32_32];
A_kron_hardest = [-gh*kron22_22 + eye(length(kron22_22)), -gh*(kron22_32 + kron32_22) -gh*kron32_32];


%% Mapping uniquee elements of H
% Easiest
[u_uniquee, ~, idx] = unique(vecH, 'stable');
vecH_len = length(vecH);
u_len = length(u_uniquee);
S = zeros(vecH_len, u_len);


% new
[u_uniquee_new, ~, idx_new] = unique(vecH_new, 'stable');
vecH_len_new = length(vecH_new);
u_len_new = length(u_uniquee_new);
S_new = zeros(vecH_len_new, u_len_new);

% Hardest
[u_uniquee_hardest, ~, idx_hardest] = unique(vecH_hardest, 'stable');
vecH_len_hardest = length(vecH_hardest);
u_len_hardest = length(u_uniquee_hardest);
S_hardest = zeros(vecH_len_hardest, u_len_hardest);

%% Fullfil S
% Easiest
for i = 1:vecH_len
    S(i, idx(i)) = 1;
end
vecH_recon = S * u_uniquee; 


for i = 1:vecH_len_new
    S_new(i, idx_new(i)) = 1;
end
vecH_recon_new = S_new * u_uniquee_new; 

% Hardest
for i = 1:vecH_len_hardest  
    S_hardest(i, idx_hardest(i)) = 1;
end
vecH_recon_hardest = S_hardest * u_uniquee_hardest; 


%% Full Solution
% Easiest
A_unic = A_kron*S;
u_p = pinv(A_unic)*b_kron;
u_n = null(A_unic);

% New
A_unic_new = A_kron_new*S_new;
u_p_new = pinv(A_unic_new)*b_kron_new;
u_n_new = null(A_unic_new);

% Hardest
A_unic_hardest = A_kron_hardest*S_hardest;
u_p_hardest = pinv(A_unic_hardest)*b_kron_hardest;
u_n_hardest = null(A_unic_hardest);


% %% Alternative method (SVD)
% [U, S, V] = svd(A_unic);
% tol = max(size(A_unic)) * eps(norm(A_unic));
% r = sum(diag(S) > tol); % rank da matriz
% 
% % Solução particular (mínimos quadrados)
% x_particular = V(:,1:r) * inv(S(1:r,1:r)) * U(:,1:r)' * b_kron  ;
% 
% % Base do nullspace
% N = V(:,r+1:end);

%% Pivot matrix (u_n -> theta)

P = zeros(36);
indices = [1,1; 2,3; 3,16; 4,21; 5,25; 6,28; 7,30; 8,36; ...
           9,2; 10,4; 11,5; 12,6; 13,7; 14,8; 15,14; ...
           16,9; 17,10; 18,11; 19,12; 20,13; 21,15; ...
           22,17; 23,18; 24,19; 25,20; 26,31; 27,22; ...
           28,23; 29,24; 30,32; 31,26; 32,27; 33,33; ...
           34,29; 35,34; 36,35];

for i = 1:size(indices,1)
    P(indices(i,1), indices(i,2)) = 1;
end


alpha = pinv(S*u_n) * (vecH - S*u_p);
u_solutuion = u_p + u_n*alpha;
changed_u = P*u_solutuion; % Changed_u must be equal to theta modifield (without 2 * h_{ij})


changed_u_p = P*u_p;
changed_u_n = P*u_n;
changed_alpha = pinv(changed_u_n) * (changed_u - changed_u_p); 

changed_u_solution = changed_u_p + changed_u_n * changed_alpha;


% h_hat = F.FromTHETAtoP(changed_u_solution,8);

rank(h_hat);

% checking if it is right
changed_u_solution - changed_u;

theta_star = changed_alpha;  
THETA0factor = 0.8;
theta_hat = THETA0factor*theta_star;  
p = 10*eye(length(theta_hat),length(theta_hat));  

t = 0:0.01:50;
% fi= [sin(t);
%     sin(2*t)];
% c = (fi'*theta_star)';  
% 
x0 = xm0x;
y = [Cmx*xm0x];
x = [xm0x];
for i = 1:length(t)
    xm = Amx*x0;
    x = [x xm];
    y= [y Cmx*xm];
   
    x0 = xm;

end

% plot(t, y);

for i = 3:length(y)
    aug_state = [y(i-2);(y(i-1) - y(i-2))/h; x(:, i-2)];
    next_aug_state = [y(i-1);(y(i) - y(i-1))/h; x(:, i-1)];

    u = -Kx * aug_state;
    next_u = -Kx * next_aug_state;

    L = [aug_state; u];
    next_L = [next_aug_state; next_u];

    reward = - aug_state'*Q_LQR*aug_state - u'*R*u;

    bar_L = F.Fromx2xbar(L);
    next_bar_L = F.Fromx2xbar(next_L);

    fi = bar_L -gh*next_bar_L;
    error = reward - fi'*theta;
end



% e = c(:,i) - fi(:,i)'*theta_hat;
% theta_hat = theta_hat + (p*fi(:,i)*e)/(1 + fi(:,i)'*p*fi(:,i));
% 
% p = p - (p*fi(:,i)*fi(:,i)'*p)/(1 + fi(:,i)'*p*fi(:,i));
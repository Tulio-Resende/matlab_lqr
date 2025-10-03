syms ts real


%% Sampling time
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

np=max(size(Ac));


% Método 1: Definir variáveis simbólicas individualmente
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

H = [H11, H12, H13; H21, H22, H23; H31, H32, H33]


% Matriz A 8x8 simétrica (se for o caso)
syms a11 a12 a13 a14 a15 a16 a17 a18
syms a21 a22 a23 a24 a25 a26 a27 a28
syms a31 a32 am11 am12 am13 am14 am15 a38
syms a41 a42 am21 am22 am23 am24 am25 a48
syms a51 a52 am31 am32 am33 am34 am35 a58
syms a61 a62 am41 am42 am43 am44 am45 a68
syms a71 a72 am51 am52 am53 am54 am55 a78
syms a81 a82 a83 a84 a85 a86 a87 a88


K = [k1, k2 km1, km2, km3, km4, km5];
Kx = [0.526620279669378,	0.262442002550444,	-4.66660887607784,	0.0718605138143464,	-0.112756794852737,	0.00130992082259127,	-0.000285134518981841];


A = [a11, a12, a13, a14, a15, a16, a17, a18;
     a21, a22, a23, a24, a25, a26, a27, a28;
     a31, a32, am11, am12, am13, am14, am15, a38;
     a41, a42, am21, am22, am23, am24, am25, a48;
     a51, a52, am31, am32, am33, am34, am35, a58;
     a61, a62, am41, am42, am43, am44, am45, a68;
     a71, a72, am51, am52, am53, am54, am55, a78;
     a81, a82, a83, a84, a85, a86, a87, a88];

% Particionar A em blocos
A11 = A(1:2, 1:2);    % Aa (planta desconhecida)
A12 = zeros(2,5);    % 
A13 = Bc;
A21 = zeros(5, 2);
A22 = Amx;    % K*Aa
A23 = zeros(5,1);    % Am (conhecida)
A31 = Kx(1:2) * A11;
A32 = Kx(3:7)*A22;
A33 = Kx(1:2)*Bc;

% syms A11 A12 A13 A21 A22 A23 A31 A32 A33

A = [A11, A12, A13; A21, A22, A23; A31, A32, A33]




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
Q = [Q11, Q12, Q13; Q21, Q22, Q23; Q31, Q32, Q33]

% 4. EQUAÇÃO DE LYAPUNOV EM BLOCOS: A'*H*A - H + Q = 0

fprintf('=== EQUAÇÕES DE LYAPUNOV POR BLOCO ===\n\n');

% Bloco (1,1): A11'*H11*A11 + A11'*H12*A21 + A21'*H21*A11 + A21'*H22*A21 - H11 + Q11 = 0
fprintf('=== BLOCO (1,1) ===\n');
eq11 = A11'*H11*A11 + A11'*H12*A21 + A21'*H21*A11 + A21'*H22*A21 - H11 + Q11;
disp('A11''*H11*A11 + A11''*H12*A21 + A21''*H21*A11 + A21''*H22*A21 - H11 + Q11 = 0');
disp('Equação simplificada:');
disp(simplify(eq11) == 0);
fprintf('\n');

% Bloco (1,2): A11'*H11*A12 + A11'*H12*A22 + A21'*H21*A12 + A21'*H22*A22 - H12 + Q12 = 0
fprintf('=== BLOCO (1,2) ===\n');
eq12 = A11'*H11*A12 + A11'*H12*A22 + A21'*H21*A12 + A21'*H22*A22 - H12 + Q12;
disp('A11''*H11*A12 + A11''*H12*A22 + A21''*H21*A12 + A21''*H22*A22 - H12 + Q12 = 0');
disp('Equação simplificada:');
disp(simplify(eq12) == 0);
fprintf('\n');

% Bloco (1,3): A11'*H11*A13 + A11'*H12*A23 + A21'*H21*A13 + A21'*H22*A23 - H13 + Q13 = 0
fprintf('=== BLOCO (1,3) ===\n');
eq13 = A11'*H11*A13 + A11'*H12*A23 + A21'*H21*A13 + A21'*H22*A23 - H13 + Q13;
disp('A11''*H11*A13 + A11''*H12*A23 + A21''*H21*A13 + A21''*H22*A23 - H13 + Q13 = 0');
disp('Equação simplificada:');
disp(simplify(eq13) == 0);
fprintf('\n');

% Bloco (2,2): A12'*H11*A12 + A12'*H12*A22 + A22'*H21*A12 + A22'*H22*A22 - H22 + Q22 = 0
fprintf('=== BLOCO (2,2) ===\n');
eq22 = A12'*H11*A12 + A12'*H12*A22 + A22'*H21*A12 + A22'*H22*A22 - H22 + Q22;
disp('A12''*H11*A12 + A12''*H12*A22 + A22''*H21*A12 + A22''*H22*A22 - H22 + Q22 = 0');
disp('Equação simplificada:');
disp(simplify(eq22) == 0);
fprintf('\n');

% Bloco (2,3): A12'*H11*A13 + A12'*H12*A23 + A22'*H21*A13 + A22'*H22*A23 - H23 + Q23 = 0
fprintf('=== BLOCO (2,3) ===\n');
eq23 = A12'*H11*A13 + A12'*H12*A23 + A22'*H21*A13 + A22'*H22*A23 - H23 + Q23;
disp('A12''*H11*A13 + A12''*H12*A23 + A22''*H21*A13 + A22''*H22*A23 - H23 + Q23 = 0');
disp('Equação simplificada:');
disp(simplify(eq23) == 0);
fprintf('\n');

% Bloco (3,3): A13'*H11*A13 + A13'*H12*A23 + A23'*H21*A13 + A23'*H22*A23 - H33 + Q33 = 0
fprintf('=== BLOCO (3,3) ===\n');
eq33 = A13'*H11*A13 + A13'*H12*A23 + A23'*H21*A13 + A23'*H22*A23 - H33 + Q33;
disp('A13''*H11*A13 + A13''*H12*A23 + A23''*H21*A13 + A23''*H22*A23 - H33 + Q33 = 0');
disp('Equação simplificada:');
disp(simplify(eq33) == 0);

% 5. SIMPLIFICAÇÕES IMPORTANTES
fprintf('\n=== SIMPLIFICAÇÕES ===\n');
fprintf('A12 = zeros(2,5) → muitos termos se anulam\n');
fprintf('A21 = zeros(5,2) → muitos termos se anulam\n');
fprintf('A31 = zeros(1,2), A32 = zeros(1,5)\n');
fprintf('Q13 = zeros(2,1), Q23 = zeros(5,1)\n\n');

% 6. EQUAÇÕES SIMPLIFICADAS
fprintf('=== EQUAÇÕES SIMPLIFICADAS ===\n');

% Bloco (1,1) simplificado
fprintf('BLOCO (1,1): A11''*H11*A11 - H11 + Q11 = 0\n');
eq11_simple = A11'*H11*A11 - H11 + Q11;
disp('Equação:');
disp(eq11_simple == 0);
fprintf('\n');

% Bloco (1,2) simplificado  
fprintf('BLOCO (1,2): A11''*H12*A22 - H12 + Q12 = 0\n');
eq12_simple = A11'*H12*A22 - H12 + Q12;
disp('Equação:');
disp(eq12_simple == 0);
fprintf('\n');

% Bloco (1,3) simplificado
fprintf('BLOCO (1,3): A11''*H11*A13 + A11''*H12*A23 - H13 = 0\n');
eq13_simple = A11'*H11*A13 + A11'*H12*A23 - H13;
disp('Equação:');
disp(eq13_simple == 0);
fprintf('\n');

% Bloco (2,2) simplificado
fprintf('BLOCO (2,2): A22''*H22*A22 - H22 + Q22 = 0\n');
eq22_simple = A22'*H22*A22 - H22 + Q22;
disp('Equação:');
disp(eq22_simple == 0);
fprintf('\n');

% Bloco (2,3) simplificado
fprintf('BLOCO (2,3): A22''*H22*A23 - H23 = 0\n');
eq23_simple = A22'*H22*A23 - H23;
disp('Equação:');
disp(eq23_simple == 0);
fprintf('\n');

% Bloco (3,3) simplificado
fprintf('BLOCO (3,3): A13''*H11*A13 + A13''*H12*A23 + A23''*H22*A23 - H33 + R = 0\n');
eq33_simple = A13'*H11*A13 + A13'*H12*A23 + A23'*H22*A23 - H33 + R;
disp('Equação:');
disp(eq33_simple == 0);



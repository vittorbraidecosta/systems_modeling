%% Código seguidor

% Espaço de Estados
A = readmatrix('A.txt');
B1l = readmatrix('B1.txt');
B1 = [zeros(6,3) B1l zeros(6,1)];
B2 = readmatrix('B2.txt');
% entradas controladas
C = [1 0 0 0 0 0; 
     0 1 0 0 0 0; 
     0 0 1 0 0 0]; % Matriz de observações
D = zeros(size(C,1),size(B2,2));

%% Seguidor de referência constante
Lambda = [A B2; C D]; % Matriz para cálculo de Nx e Nu
No = [zeros(1, length(A)) ones(1, length(C(:,1)))]';
NxNu = inv(Lambda)*No; % Vetores Nu e Nx
Nx = NxNu(1:length(A)); % Vetor Nx
Nu = NxNu((length(A)+1):length(NxNu)); % Vetor Nu

p = 0.2*[-1 + 0.8i; -1 - 0.8i; -1; -2; -1 + 0.7i; -1 - 0.7i]; % polos
K = place(A, B2, p); % Matriz de ganhos
F = A - B2*K; % Matriz de malha fechada

urp = Nu + K*Nx; % Lei de controle em RP
xr = [0; 0; 0; 1; 2; pi/4]; % Referência
Bn = xr.*B2*urp; % Vetor de entradas com Lei de controle
Cn1 = [1 0 0 0 0 0];
Dn = zeros(size(Cn1,1),size(Bn,2));

Tf = 35; % Tempo final (s)
dt = 0.01; % Intervalo de tempo (s)
t = 0:dt:Tf; % Vetor de tempos
sysrc1 = ss(F, Bn, Cn1, Dn);
[ysc1, tsc1, xsc1] = step(sysrc1, t); % Step a ser seguido

% Cálculo das forças do seguidor
Nxm = [1 0 0;
       0 1 0;
       0 0 1;
       0 0 0;
       0 0 0;
       0 0 0]; % Matriz Nx
Num = [Nu(1) 0 0;
       0 Nu(2) 0;
       0 0 Nu(3)]; % Matrix Nu
xrc = xr(4:6);
for i = 1:(length(t)) % Forças atuadores
    tau(:,i) = K*xsc1(i,:)'-(K*Nxm+Num)*xrc;
end 
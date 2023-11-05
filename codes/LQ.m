%% Regulador LQ

% Espaço de Estados
A = readmatrix('A.txt');
B1 = readmatrix('B1.txt');
B2 = readmatrix('B2.txt');
C = readmatrix('C.txt');
D = zeros(size(C,1),size(B1,2));

% Implementação do LQR
Q = [8000 0 0 0 0 0; 
     0 2000 0 0 0 0; 
     0 0 8000 0 0 0;
     0 0 0 1000 0 0;
     0 0 0 0 1000 0;
     0 0 0 0 0 1000]; % Matriz de pesos para estados
R = [.6 0 0; 
     0 .8 0; 
     0 0 .2]; % Matriz de pesos para as entradas
[Klq, S, P] = lqr(A,B2,Q,R); % obtenção de Ganho (K)

Flq = A - B2*Klq; % Matriz de malha fechada para LQR
plq = eig(Flq); % Polos da matriz de malha fechada para LQR

%% Dados para matriz de transição
dt = 0.1; % Intervalo de tempo (s)
Tf = 20; % Tempo final de simulção (s)
t = 0:dt:Tf; % Vetor de tempo

Phi_Flq = expm(Flq*dt); % Matriz de transição do LQR
Gama_Flq= eye(size(Flq,1))*dt + Flq*dt^2/2 + Flq^2*dt^3/6 + Flq^3*dt^4/24;

x0 = [1; 2; 0.5; 0; 0; 0]; % Condições iniciais
xlq = zeros(6,length(t)); % pré-alocação do vetor x
xlq(:,1) = x0; % Inserindo condições iniciais no vetor x
for i = 1:(length(t)-1) % loop para simulação
    xlq(:,i+1) = Phi_Flq*xlq(:,i);
end

for i = 1:(length(t)) % Cálculo das forças do regulador de cond inic
    tau(:,i) = Klq*xlq(:,i);
end 

%% Entrada de distúrbio
sysc = ss(Flq, B1, C, D); % Sistema controlado pelo LQR
sysnc = ss(A, B1, C, D); % Sistema em malha aberta
[ysc, tsc, xsc] = step(sysc, t); % Entrada degrau de corrente
[ysnc, tsnc, xsnc] = step(sysnc, t); % Entrada degrau de corrente

for i = 1:(length(t)) % Cálculo das forças do regulador para corrente
    tau2(:,i) = Klq*xsc(i,:,1)';
end
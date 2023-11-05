%Matrizes no espaco de estados
A=readmatrix("A.txt");
B1=readmatrix("B1.txt");
B2=readmatrix("B2.txt");
C=readmatrix("C.txt");
D=zeros(size(C,1),size(B1,2));
%Vetor de polos
p=[-0.2;-0.2;-0.25+0.25*1i;-0.25-0.25*1i;-0.3;-0.4];
%Alocacao de polos
K=place(A,B2,p);
%Malha aberta
sysma=ss(A,B1,C,D);
%Malha fechada
F=A-B2*K;
sysmf=ss(F,B1,C,D);
%Matriz de transicao
dt=0.01;
tfmt=35;
t=0:dt:tfmt;
x0=[1;2;0.5;0;0;0];
x=zeros(6,length(t));
x(:,1)=x0;
Phi=expm(F*dt);
for i=1:length(t)-1
    x(:,i+1)=Phi*x(:,i);
end
%Degrau
tfd=30;
[yma,tma,xma]=step(sysma,tfd);
[ymf,tmf,xmf]=step(sysmf,tfd);
%Esforços dos atuadores
for i = 1:(length(t)) % Cálculo das forças do regulador para condições iniciais
    taumt(:,i) = K*x(:,i);
end
for j = 1:(length(tmf)) % Cálculo das forças do regulador para corrente
    taumf(:,j) = K*xmf(j,:,1)';
end

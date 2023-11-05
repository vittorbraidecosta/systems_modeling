%% Seguir de referência variável

%% 1. Sistema
A = readmatrix('A.txt');
B1 = readmatrix('B1.txt');
B2 = readmatrix('B2.txt');
C = readmatrix('C.txt');
D = 0;
%% 2. Referência e distúrbios 
om=2*pi/1000;
Ar = [0 1 0 0 0 0;
      -om^2 0 0 0 0 0;
      0 0 0 1 0 0;
      0 0 -om^2 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0];
Aw = [0 0 0 1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0;
      0 0 0 0 0 0];

%% 3. Controlador
p=0.3*[-1+1i; -1-1i; -1+.8i;-1-.8i;-1.5;-1];
K=place(A,B2,p); % Ganhos de realimentação
F=A-B2*K; % Fechando a malha

%% 4. Matrizes do seguidor 
F2=[(A-Ar) F];
Ao=[Ar zeros(6);zeros(6) Aw];
Kex=inv(C*inv(F)*B2)*C*inv(F)*F2;
Ab=[F F2-B2*Kex;zeros(12,6) Ao];

%% 5. Simulação por matriz de transferência 
%Vetor de tempos
dt = 0.01;
t = 0:dt:2000;
%Condições iniciais 
x0=[0;0;0;0;0;0];
xr=12*[sin(om*t);om*cos(om*t);cos(om*t);-om*sin(om*t);om*t;om*ones(1,length(t))];
xex=[xr;zeros(size(xr))];
e=zeros(6,length(t));
e(:,1)=x0-xr(:,1);
X=[e;xex];
Phi=expm(Ab*dt);
for i=2:length(t)
 X(:,i) = Phi*X(:,i-1);
end
x = X(1:6,:)+X(7:12,:);

%% 6. Construção dos gráficos 
fig=figure(1);
plot(x(1,:),x(3,:),'-','LineWidth',2);
hold on
plot(xr(1,:),xr(3,:),'--','LineWidth',2);
xlim([-15 15])
ylim([-15 15])
title('Trajetória com seguidor de referência variável sem distúrbios')
xlabel('Posição X (m)')
ylabel('Posição Y (m)')
legend('Trajetória realizada','Trajetória desejada')
grid on 
print(fig,'seguidor_variavel','-dpng','-r300')

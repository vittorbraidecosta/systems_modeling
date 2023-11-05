%% 1. Sistema 

A=readmatrix("A.txt");
B2=readmatrix("B2.txt");
B1=readmatrix("B1.txt");
C=readmatrix("C.txt");
D=0;

%Polos dos sistema 
p=eig(A);

%% 2. Controladores

%Alocação 
p_ctr=[-0.2;-0.2;-0.25+0.25*1i;-0.25-0.25*1i;-0.3;-0.4];
K_ctr=place(A,B2,p_ctr); %Vetor de ganhos para alocação
F_ctr=A-B2*K_ctr; % Matriz de malha fechada para LQR

%Controlador LQ
Q = [300 0 0 0 0 0; 
     0 200 0 0 0 0; 
     0 0 200 0 0 0;
     0 0 0 100 0 0;
     0 0 0 0 100 0;
     0 0 0 0 0 100]; % Matriz de pesos para estados
P = [1 0 0; 
     0 0.5 0; 
     0 0 1]; % Matriz de pesos para as entradas
K_ctr_lq=lqr(A,B2,Q,P); % obtenção de Ganho (K)

F_ctr_lq=A-B2*K_ctr_lq; % Matriz de malha fechada para LQR
p_ctr_lq= eig(F_ctr_lq); % Polos da matriz de malha fechada para LQR

%% 3. Observadores 
%Observador por alocação
p_obs=[-0.45;-0.55;-0.65;-0.75;-0.85;-0.95]; %todos os polos a esquerda dos polos de MF e sem oscilações 
K_obs=place(A',C',p_obs); 

%Observador por LQR
Qo = [300 0 0 0 0 0; 
     0 200 0 0 0 0; 
     0 0 200 0 0 0;
     0 0 0 200 0 0;
     0 0 0 0 200 0;
     0 0 0 0 0 200]; % Matriz de pesos para estados
Po = [3.5 0 0; 
     0 3.5 0; 
     0 0 3.5]; % Matriz de pesos para as entradas
K_obs_lq=lqr(A',C',Qo,Po);
F_obs_lq=A'-C'*K_obs_lq;
p_obs_lq=eig(F_obs_lq);

%Plot plano imaginário 

imag_ctr=imag(p_ctr);
real_ctr=real(p_ctr); 
imag_obs=imag(p_obs);
real_obs=real(p_obs);
mapa=figure(10);
AxesH = axes('Xlim', [-1, 0], 'XTick', -1:0.2:0,'Ylim', [-0.3, 0.3], 'YTick', -0.3:0.1:0.3,'NextPlot', 'add');
plot(real_ctr,imag_ctr,"x",real_obs,imag_obs,"o",'MarkerSize', 8)
legend('Polos do Controlador','Polos do Observador','Location','northwest')
title('Polos do controlador e observador obtidos por alocação')
xlabel('Eixo Real')
ylabel('Eixo Imaginário')
grid on
print(mapa,'mapa_alocacao','-dpng','-r300') 

imag_ctr=imag(p_ctr_lq);
real_ctr=real(p_ctr_lq); 
imag_obs=imag(p_obs_lq);
real_obs=real(p_obs_lq);
mapa=figure(20);
plot(real_ctr,imag_ctr,"x",real_obs,imag_obs,"o",'MarkerSize', 8)
legend('Polos do Controlador','Polos do Observador','Location','northwest')
title('Polos do controlador e observador LQ')
xlabel('Eixo Real')
ylabel('Eixo Imaginário')
grid on
print(mapa,'mapa_lqr','-dpng','-r300')
%% 4. Simulação 

%Vetor de tempos 
t=0:0.1:30;

%Observador por alocação 
Lambda=[A,-B2*K_ctr;K_obs'*C,A-B2*K_ctr-K_obs'*C];

%Distúbios
Uce=1*ones(size(t));
Vce=1*ones(size(t));
w=[Uce;Vce];

%Condições iniciais 
x0=[0;0;0;0;0;0];
x0_hat=[1;1;1;1;1;1.1];
z0=[x0;x0_hat];
sys_obs=ss(Lambda,[B1;B1],eye(length(Lambda)),0);
y=lsim(sys_obs,w,t,z0);
plot_observer(t,y,'alocacao');

%Observador por LQ 
Lambda_lqr=[A,-B2*K_ctr_lq;K_obs_lq'*C,A-B2*K_ctr_lq-K_obs_lq'*C];
sys_obs_lqr=ss(Lambda_lqr,[B1;B1],eye(length(Lambda)),0);
y_lqr=lsim(sys_obs_lqr,w,t,z0);
plot_observer(t,y_lqr,'lqr');


figure(30)
L=tiledlayout(1,3);
ax1=nexttile;
plot(ax1,t,y(:,1)-y(:,7),'LineWidth',2)
grid on
ax2=nexttile; 
plot(ax2,t,y(:,2)-y(:,8),'LineWidth',2)
grid on
ax3=nexttile;
plot(ax3,t,y(:,3)-y(:,9),'LineWidth',2)
grid on
linkaxes([ax1,ax2,ax3],'x');
xlabel(L,'Tempo (s)','FontSize',18)
ylabel(L,'Amplitude','FontSize',18)
title(L,'Evolução temporal do erro para as variáveis de posição','FontSize',18)
ylim(ax3,[0 1])
title(ax1,'Erro da variável x','FontSize',18)
title(ax2,'Erro da variável y','FontSize',18)
title(ax3,'Erro da variável \psi','FontSize',18)
L.TileSpacing = 'compact';
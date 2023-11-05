%% SINTESE PID POR ZN 
%Sistema
A = readmatrix("A.txt");
B1 = readmatrix("B1.txt");
B2 = readmatrix("B2.txt");
B=[B2,B1]; 
C = readmatrix("C.txt");
D_sys = zeros(size(C,1),size(B,2));

%Definindo a FT
[n,D]=ss2tf(A,B,C,D_sys,1); %[numerador, denominador] da FT
N=n(1,:);
G=tf(N,D);

% %Mapa de zeros e polos para FTMA
% [z,p,k]=tf2zp(N,D);
% imag_ctr=imag(p);
% real_ctr=real(p); 
% imag_obs=imag(z);
% real_obs=real(z);
% mapa=figure(10);
% AxesH = axes('Xlim', [-0.2, 0], 'XTick', -0.2:0.05:0,'Ylim', [-0.8, 0.8], 'YTick', -0.8:0.2:0.8,'NextPlot', 'add');
% plot(real_ctr,imag_ctr,"x",real_obs,imag_obs,"o",'MarkerSize', 12)
% legend('Polos','Zeros','Location','northwest')
% title('Polos e zeros da Função de Transferência em Malha Aberta')
% xlabel('Eixo Real')
% ylabel('Eixo Imaginário')
% set(findall(gcf,'type','line'),'linewidth',1.5)
% grid on
% print(mapa,'mapa_ftma','-dpng','-r300') 

%Definindo Kcr
Kcr=margin(G);

%Definindo Pcr
Gcr = pid(Kcr); 
Tcr = feedback(Gcr*G,1); 
t = 0:0.01:400; 
[y, t] = step(Tcr, t);
f1 = figure;
plot(t,y,'LineWidth',2)
ylabel('Posição x (m)')
xlabel('Tempo (s)')
title('Resposta ao degrau para $K_{cr}$', 'Interpreter', 'Latex')
grid on
print(f1,'respDegKcr.png','-dpng','-r300')
Pcr=59.4;

%Ganhos 
Kp=0.6*Kcr; 
Ki=Kp/(0.5*Pcr);
Kd=0.125*Pcr*Kp;

%Controlador
Gc = tf([Kd Kp Ki],[1 0]); 

T = feedback(Gc*G,1); % FTMF

t = 0:0.01:800; 
[ypid, tpid] = step(T, t); 
f2 = figure;
plot(tpid, ypid,'LineWidth',2)
ylabel('Posição x (m)')
xlabel('Tempo (s)')
title('Resposta com controlador PID', 'Interpreter', 'Latex')
grid on
print(f2,'respDegPID_ZN.png','-dpng','-r300')

%Esforços de Atuação
u=acao(ypid,tpid,1,Kp,Ki,Kd);
f10=figure;
plot(tpid, u,'LineWidth',2)
ylabel('Ação de controle x (N)')
xlabel('Tempo (s)')
title('Ação de controle para Ziegler-Nichols')
grid on
print(f10,'acao_ZN.png','-dpng','-r300')

%Diagrama de Bode
f4 = figure;
margin(Gc*G)
grid on 
print(f4,'bode_ZN.png','-dpng','-r300')
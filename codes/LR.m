clear;clc;close all

%% Atividade - Controlador PID - Lugar das raízes

%% Função de transferência utilizada
N = 10^(-3)*[2.218 .2024 .2528 .003805 0]; % Numerador
D = [1 0.273796 .157203 .00401842 .000090883 0 0]; % Denominador
GH1 = tf(N,D); % Função de transferência

%f11 = figure;
%margin(GH1)
%grid on
%axis([10^(-2) 1 10 10])

%% Simulação para degrau
%[y, t] = step(GH1, 400); % Degrau unitário
%{
f1 = figure;
plot(t,y,'LineWidth',2)
ylabel('Posição x (m)')
xlabel('Tempo (s)')
title('Resposta ao degrau do movimento translacional pela entrada $\tau_u$',...
    'Interpreter', 'Latex')
grid on
print(f1,'respDeg.png','-dpng','-r300')
%}
%% Lugar das raízes GH_1
%{
f2 = figure;
rlocus(GH1) % Criando o lugar das raízes
%set(findall(gcf,'type','line'),'linewidth',2);
axis([-0.2 0.05 -1 1])
title('Lugar das raízes')
xlabel('Eixo real')
ylabel('Eixo imaginário')
%print(f2,'lugRaizMA.png','-dpng','-r300')
%}
%% Controlador P
Kp = 1.11; % Constante de ganho Proporcional P
Gc_P = Kp; % Função de transferência do controlador P
T_P = feedback(GH1, Gc_P); % FT em malha fechada do controlador P
[y_cp, t_cp] = step(T_P, 800); % Degrau unitário para sistema controlado (P)

%{
f3 = figure;
plot(t_cp,y_cp,'LineWidth',2)
ylabel('Posição x (m)')
xlabel('Tempo (s)')
title('Resposta ao degrau com controlador P')
grid on
print(f3,'respDegP.png','-dpng','-r300')
%}

%% Lugar das raízes GH_2
Ns = 10^(-3)*[0 0 2.218 .2024 .2528 .003805 0 0]; % Numerador
Ds = [1 0.273796 .157203 .00401842 .000090883 0 0 0]; % Denominador
GH2 = tf(N,Ds+Kp*Ns); % Função de transferência para Lugar das Raízes
%{
f4 = figure;
rlocus(GH2) % Criando o lugar das raízes
%set(findall(gcf,'type','line'),'linewidth',2);
axis([-0.2 0.2 -0.5 0.5])
title('Lugar das raízes')
xlabel('Eixo real')
ylabel('Eixo imaginário')
%print(f2,'lugRaizMA2.png','-dpng','-r300')
%}
%% Controlador PI
%Ki = 0.00115; % Ganho integral
%Ki = 0.00448;
Ki = 0.01;
Gc_PI = tf([Kp Ki],[1 0]); % FT do controlador
T_2 = feedback(Gc_PI*GH1,1);
[y_cpi, t_cpi] = step(T_2, 1500); % Degrau unitário para sistema controlado (P)
%{
f5 = figure;
plot(t_cpi,y_cpi,'LineWidth',2)
ylabel('Posição x (m)')
xlabel('Tempo (s)')
title('Resposta ao degrau com controlador PI')
grid on
print(f5,'respDegPI.png','-dpng','-r300')
%}
%% Lugar das raízes GH_3
Ns2 = 10^(-3)*[0 2.218 .2024 .2528 .003805 0 0 0]; % Numerador
Nden = 10^(-3)*[0 0 0 2.218 .2024 .2528 .003805 0]; % Numerador
Ds = [1 0.273796 .157203 .00401842 .000090883 0 0 0]; % Denominador
GH3 = tf(Ns2,Ds+Kp*Ns+Ki*Nden); % Função de transferência para Lugar das Raízes
%{
f6 = figure;
rlocus(GH3) % Criando o lugar das raízes
%set(findall(gcf,'type','line'),'linewidth',2);
axis([-0.15 0.02 -0.4 0.4])
title('Lugar das raízes');
xlabel('Eixo real');
ylabel('Eixo imaginário')
%}
%% Controlador PID
Kd = 5.92; % Ganho derivativo
Gc_PID = tf([Kd Kp Ki],[1 0]); % FT do PID
T_3 = feedback(Gc_PID*GH1,1); % FT em malha fechada do PID
t = 0:0.01:600; % vetor tempo
[y_cpid, t_cpid] = step(T_3, t); % Degrau unitário para sistema controlado (P)

f6 = figure;
plot(t_cpid,y_cpid,'LineWidth',2)
ylabel('Posição x (m)')
xlabel('Tempo (s)')
title('Resposta ao degrau com controlador PID')
grid on
%print(f6,'LRrespDegPID.png','-dpng','-r300')

%% Compensador com PID
GH4 = Gc_PID*GH1; % FT PID + Planta

%f7 = figure;
%margin(GH4) % Estabilidade relativa FTMA
%grid on

%f8 = figure;
%pzplot(T_3)

Phi = (35 + 5)*pi/180; % Fase a vançar do compensador
alpha = (1+sin(Phi))/(1-sin(Phi)); % Razão de avanço
%alpha = 6;
v = -10*log10(alpha); % Valor para determinar om_n
om_n = 0.1; % Nova frequência de corte
p = alpha*om_n/sqrt(alpha); % Polo do compensador
z = om_n/sqrt(alpha); % Zero do compensador
Gc = alpha*tf([1 z],[1 p]); % FT compensador

GH5 = Gc*Gc_PID*GH1; % FT Compensador + PID + Planta

%f8 = figure;
%margin(GH5) % Estabilidade relativa FTMF
%grid on

T_4 = feedback(GH5,1); % FTMF com compensador
t2=0:0.01:600;
[ycomp, tcomp] = step(T_4, t2); % Degrau unitário para sistema controlado (P)

%f9 = figure;
%pzplot(T_4)

f10 = figure;
plot(tcomp, ycomp, 'LineWidth', 2)
ylabel('Posição x (m)')
xlabel('Tempo (s)')
title('Resposta ao degrau com controlador PID e compensador')
grid on
%print(f10,'LRrespDegPIDeCompensador.png','-dpng','-r300')

%{
%% Ação de Controle
u1 = acao(y_cpid,t_cpid,1,Kp,Ki,Kd);
u2 = acao(y,t,1,Kp,Ki,Kd);

f11 = figure;
plot(t_cpid, u1*10, 'LineWidth', 2)
title('Ação de controle')
ylabel('Ação de controle (N)')
xlabel('Tempo (s)')
grid on
print(f11,'LREsforcosfake.png','-dpng','-r300')

f12 = figure;
plot(t, u2*10, 'LineWidth', 2)
title('Ação de controle')
ylabel('Ação de controle (N)')
xlabel('Tempo (s)')
grid on
print(f12,'LRCompEsforcosfake.png','-dpng','-r300')
%}
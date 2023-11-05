clear;
plot_veloc=0;
tau=[50,0,0];
uc=0;
vc=0;
t=0:0.01:400;
x0=[0,0,0,0,0,0];
%Espaco de estados
A = readmatrix("A.txt");
B2 = readmatrix("B2.txt");
B1 = readmatrix("B1.txt");
B= [B2,B1];
C = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0;0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1];
D = zeros(size(C,1),size(B,2));
sys=ss(A,B,C,D);
u=zeros(5,length(t));
u(1,:) = tau(1).*ones(size(t));
u(2,:) =tau(2).*ones(size(t));
u(3,:) = tau(3).*ones(size(t));
u(4,:) = uc.*ones(size(t));
u(5,:) = vc.*ones(size(t));
[y,t]=lsim(sys,u,t);

%Matriz de transição
X = MatTrans(A,B,u,0.01,400,x0);
%POSIÇÃO
%Plot de x
f1 = figure;
plot(t,X(1,:),'LineWidth',2,'LineStyle','--')
hold on
plot(t,y(:,1),'LineWidth',2)
hold off
title('Movimento de translação em X')
xlabel('Tempo (segundos)')
ylabel('Coordenada x (m)')
grid on
legend('Matriz de Transição','Espaço de estados')
print(f1, 'ACPderiva.png', '-dpng', '-r300'); 
%Plot de y
f2=figure;
plot(t,X(2,:),'LineWidth',2,'LineStyle','--')
hold on
plot(t,y(:,2),'LineWidth',2)
hold off
title('Movimento de translação em Y')
xlabel('Tempo (segundos)')
ylabel('Coordenada y (m)')
grid on
legend('Matriz de Transição','Espaço de estados')
print(f2, 'DCPderiva.png', '-dpng', '-r300'); 
%Plot de r 
f3=figure;
plot(t,X(3,:),'LineWidth',2,'LineStyle','--')
hold on
plot(t,y(:,3),'LineWidth',2)
hold off
title('Movimento de rotação')
xlabel('Tempo (segundos)')
ylabel('Ângulo, \psi [rad]','interpreter','tex')
grid on
legend('Matriz de Transição','Espaço de estados')
print(f3, 'GCPderiva.png', '-dpng', '-r300'); 

if plot_veloc==1
    %VELOCIDADE
    %Plot de xdot
    f1 = figure;
    plot(t,X(4,:),'LineWidth',2,'LineStyle','--')
    hold on
    plot(t,y(:,4),'LineWidth',2)
    hold off
    title('Velocidade de translação em X')
    xlabel('Tempo (segundos)')
    ylabel('Velocidade $\dot{x}$ (m/s)','interpreter','latex')
    grid on
    legend('Matriz de Transição','Espaço de estados')
    print(f1, 'veloc_ACPderiva.png', '-dpng', '-r300'); 
    %Plot de y
    f2=figure;
    plot(t,X(5,:),'LineWidth',2,'LineStyle','--')
    hold on
    plot(t,y(:,5),'LineWidth',2)
    hold off
    title('Velocidade de translação em Y')
    xlabel('Tempo (segundos)')
    ylabel('Velocidade $\dot{y}$ (m/s)','interpreter','latex')
    grid on
    legend('Matriz de Transição','Espaço de estados')
    print(f2, 'veloc_DCPderiva.png', '-dpng', '-r300'); 
    %Plot de r 
    f3=figure;
    plot(t,X(6,:),'LineWidth',2,'LineStyle','--')
    hold on
    plot(t,y(:,6),'LineWidth',2)
    hold off
    title('Velocidade de rotação')
    xlabel('Tempo (segundos)')
    ylabel('Velocidade $\dot{\psi}$ (rad/s)','interpreter','latex')
    grid on
    legend('Matriz de Transição','Espaço de estados')
    print(f3, 'veloc_GCPderiva.png', '-dpng', '-r300'); 
end 



function x = MatTrans(A,B,u,dt,Tf,x0)
    % A: Matriz do sistema
    % B: Matriz de entradas
    % u: vetor de entradas
    % dt: intervalo de tempo para cálculo das matrizes
    % Tf: tempo total de simulação
    % x0: Condições iniciais
    
    % Matriz de transição (dt)
    Phi = eye(size(A,1)) + A*dt + A^2*dt^2/2 + A^3*dt^3/6;

    % Matriz dos termos forçantes
    Gama = eye(size(A,1))*dt + A*dt^2/2 + A^2*dt^3/6 + A^3*dt^4/24;

    nt = round(Tf/dt);
    t = 0:dt:(nt*dt);
    x = zeros(6,nt);
    x(:,1) = x0;

    % Cálculo de x
    for i = 1:(nt)
        x(:,i+1) = Phi*x(:,i) + Gama*B*u(:,i);
    end
end




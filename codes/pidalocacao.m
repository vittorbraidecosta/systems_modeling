%Matrizes no espaco de estados
A=readmatrix("A.txt");
B1=readmatrix("B1.txt");
B2=readmatrix("B2.txt");
B=[B2,B1];
C=readmatrix("C.txt");
D_sys=zeros(size(C,1),size(B,2));
%Função de transferencia de malha aberta
[n,D]=ss2tf(A,B,C,D_sys,1);
N=n(1,:);
G=tf(N,D);
%Alocacao de polos
p=[-0.2+0.2*1i;-0.2-0.2*1i;-0.25+0.25*1i;-0.25-0.25*1i;-0.3;-0.3;-0.3];
s=tf('s');
eqc=(s-p(1))*(s-p(2))*(s-p(3))*(s-p(4))*(s-p(5))*(s-p(6))*(s-p(7));
[N_eqc,D_eqc]=tfdata(eqc,'v');
%MMQ
Nf=flip(N);
Df=flip(D);
N_eqcf=flip(N_eqc);
a=[Nf(1) 0 0;
    Nf(2) Nf(1) 0;
    Nf(3) Nf(2) Nf(1);
    Nf(4) Nf(3) Nf(2);
    Nf(5) Nf(4) Nf(3);
    0 Nf(5) Nf(4);
    0 0 Nf(5)];
b=[N_eqcf(1);N_eqcf(2)-Df(1);N_eqcf(3)-Df(2);N_eqcf(4)-Df(3);N_eqcf(5)-Df(4);N_eqcf(6)-Df(5);N_eqcf(7)-Df(6)];
K=lsqr(a,b);
%Controlador
Ki=K(1);
Kp=K(2);
Kd=K(3);
Gc=pid(Kp,Ki,Kd);
%Funcao de transferencia de malha fechada
U=series(Gc,G);
T=feedback(U,1);
%Reducao de ordem
[Gb,normahankel]=balreal(G);
Gr=modred(Gb,[5 6],'del');
Grg=tf(Gr);
%Simulação
tfd=100;
[y,t]=step(T,tfd);
%Ação de controle
e = 1 - y;
up = Kp*e;
de = diff(e);
dt = diff(t);
dedt  = de./dt;
dedt = [dedt; dedt(end)];
ud = Kd*dedt;
inte = cumtrapz(e,t);
ui = Ki*inte;
u=up+ud+ui;
%Plots
fig1=figure;
plot(t,y,'LineWidth',2);
title('Resposta ao degrau com controlador PID')
xlabel('Tempo (s)')
ylabel('Posição (m)')
grid on
print(fig1,'PIDalocacao.png','-dpng','-r300');
fig2=figure;
pzmap(T)
print(fig2,'poloszerosalocacao.png','-dpng','-r300');
fig3=figure;
margin(U)
print(fig3,'bodealocacao.png','-dpng','-r300');
fig4=figure;
bode(G,'b',Gr,'r')
legend('$G(s)$','$Gr(s)$','interpreter', 'latex')
print(fig4,'bodecomparacao.png','-dpng','-r300');
fig5=figure;
plot(t,u,'LineWidth',2);
title('Ação de controle')
xlabel('Tempo (s)')
ylabel('Ação de controle (N)')
grid on
print(fig5,'esforcosPIDalocacao.png','-dpng','-r300');
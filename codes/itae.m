clear all;

A = readmatrix("A.txt");
B1 = readmatrix("B1.txt");
B2 = readmatrix("B2.txt");
B=[B2,B1]; 
C = readmatrix("C.txt");
D_sys = zeros(size(C,1),size(B,2));

%Definindo a FT
[n,D]=ss2tf(A,B,C,D_sys,1); %[numerador, denominador] da FT
N=n(1,:);
sys=tf(N,D);


%Dados
wn=0.55;
Kp=676.87;
Kd=406.72;
Ki=427.96;
t=0:0.01:100

%Ganho do controlador PID 
Gc=pid(Kp,Ki,Kd);

%FTMA
GH=series(Gc,sys)


%Sistema PID
sys_itae = feedback(series(Gc,sys),1);
y_sys=step(sys_itae,t);
[num,den]=tfdata(sys_itae);


%ITAE de referencia
itae_ref=tf([wn^7],[1 2.217*wn 6.745*wn^2 9.349*wn^3 11.58*wn^4 8.68*wn^5 4.323*wn^6 wn^7]);
y_itae=step(itae_ref,t);

%esforco atuadores
u = acao(y_sys,t',1,Kp,Ki,Kd);

%Plots
figure(1)
plot(t,y_sys,LineWidth=2)
hold on
plot(t,y_itae,LineWidth=2)
grid on
xlabel("Tempo [s]")
ylabel("x [m]")
legend("PID - ITAE","ITAE referência")

figure(2)
pzmap(GH,'r')

figure(3)
margin(GH)

figure(4)
nyquist(GH)

figure(5)
plot(t,u,LineWidth=2)
grid on
xlabel("Tempo [s]")
ylabel("Fu [N]")
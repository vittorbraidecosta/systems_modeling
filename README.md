# About the Repo 
*Created on 2023.1* | This repo accommodates a work developed by 4 students from the University of São Paulo. The main goal was to build a mathematical model of the planar movement of an Autonomous Underwater Vehicle (AUV) in state space representation. Afterward, a control system was developed using modern and classical approaches with didactic purposes. 

Therefore, the repo contains all codes built in Matlab and Mathematica and also a report that documents all stages. As it was developed to a lecture in São Paulo, it is written in Portuguese.

## Codes
Here is possible to find a brief description of each code 

- **linearizacao.nb** - Mathematica code to linearize the system using Taylor Series Expansion

- **simulacoes_malhaaberta.m** - Matlab code used to simulate open loop system 

- **polos.m** - Matlab code to build the control system by pole placement and simulate the results

- **LQ.m** - Matlab code to build the control system by LQR method 

- **observadores.m** - Matlab code to add an observer for the system
 
- **SeguidorConstante.m** - Matlab code to simulate a constant reference tracking 

- **seguidor_variavel.m** - Matlab code to simulate a variable reference tracking 

- **itae.m** - Matlab code to synthesize a PID controller by ITAE method 

- **LR.m** - Matlab code to synthesize a PID controller by root-locus method 

- **pid_zn.m** - Matlab code to synthesize a PID controller by Ziegler–Nichols method

- **pid_alocacao** - Matlab code to synthesize a PID controller by pole placement method

- **nyquist.nb** - Mathematica code to build Nyquist Diagrams for Stability Analysis
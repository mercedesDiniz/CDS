%% Atividade 04: 
% Para as classes de plantas mostradas a seguir, projetar o sistemas GMVC incrementais de ordem mínima,
% realizando os seguintes testes e comparando-os com uma solução baseada em PID: 
clear all; close all; clc;

%% Planta a) G(s) = ( 1/(s+1) )*e^{-3s}
Ts_a = 1;   % periodo de amostragem (s)
d_a = round((3/Ts_a) + 1);   % atraso discreto (em amostras)

Gs_a = tf(1, [1 1])
Gz_a = c2d(Gs_a,Ts_a, 'zoh')

%% Planta b) G(s) = ( 4/(s^{2}+2s+4) )*e^{-0.3s}
Ts_b = 0.1; % periodo de amostragem (s)
d_b = round((0.3/Ts_b) + 1);   % atraso discreto (em amostras)

g = 1;          % ganho do sistema
wn = 2;         % frequencia natural (rad/s)
zeta = 0.5;     % fator de amortecimento

Gs_b = tf(g*wn^2, [1  2*zeta*wn   wn^2])
Gz_b = c2d(Gs_b,Ts_b, 'zoh')
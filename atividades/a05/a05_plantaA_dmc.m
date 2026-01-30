%% Atividade 05: Projetar e implementar, por simulação, o  PMVC, DMC e GPC.
clear all; close all; clc;

%% Parametros
Ts = 0.1;             % periodo de amostragem (s)
td = 0;                  % atraso continuo (s)
d = round((td/Ts) + 1)   % atraso discreto (em amostras)

% Nx = d+1;                % horizonte de previsão de estado (atraso virtual)

%lambida = 1;

%% Planta a) Dinâmica do ângulo de roll do VLS-1 em Max Q

% Modelo contínuo
    Gs = tf(61.0492, [1  0.098859 0]);
    
    % Verificando os autovalores
    disp('Autovalores da planta:')
    eig(Gs)

% Modelo discreto
    Gz = c2d(Gs,Ts, 'zoh')
        Az = Gz.den{1};
            a1 = Az(2); a2 = Az(3);
        Bz = Gz.num{1}
            b0 = Bz(2); b1 = Bz(3);

%% DMC





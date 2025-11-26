%% Atividade 02 - CDS
% Resolvar o problema de alocação e teste sobre a plata G(s), mostrando que uma escolha de tau_mf de projeto (diferente de 10)
% funciona com seu controle RST sintetizado;
clear all; close all; clc;
%% Planta
tau = 10;
Gs = tf(1, [ tau 1])
Ts = 1;

Gz = c2d(Gs, Ts)

% b0 = 1-exp(-Ts/tau)
% a1 = exp(-Ts/tau)

%% RST
% R(z)*u(k) = T(z)*ref(k) - S(z)*y(k)

% G_MF_desejado(z) = ( B(z)*T(z)*z^(-d) ) / ( A(z)*R(z)+B(z)*S(z)*z^(-d) )
% G_MF_desejado(z) = N(z) / Hc(z)
tau_mf = 15;
Gs_MF_desejado = tf(1, [ tau_mf 1])
Gz_MF_desejado = c2d(Gs_MF_desejado, Ts)
    % Nz = Gs_MF_desejado.num{1} 
    % Hcz = Gs_MF_desejado.den{1}

% H(z) = Hc(z)*Ho(z)
    % Polinomio desejado em M.F: Ho(z) := A(z) -> quando os polos são estaveis
    % Polinomio de "observador": Hc(z)

% T(z) = t_off*Ho(z)
    % t_off = Ho(1)/B(z) = A(1)/B(1)

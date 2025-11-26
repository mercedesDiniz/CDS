%% Atividade 02 - CDS
% Resolvar o problema de alocação e teste sobre a plata G(s), mostrando que uma escolha de tau_mf de projeto (diferente de 10)
% funciona com seu controle RST sintetizado;
clear all; close all; clc;
%% Planta
tau = 10;
Gs = tf(1, [ tau 1]) 

Ts = 1; % s
Gz = c2d(Gs, Ts) % G(z) = ( B(z)z{-1} )/A(z) = b0/( 1+a1*z{-1} )
    b0 = 1-exp(-Ts/tau)
    a1 = exp(-Ts/tau)
    

%% Comportamento Desejado
% G_MF_desejado(z) = ( B(z)*T(z)*z^(-d) ) / ( A(z)*R(z)+B(z)*S(z)*z^(-d) )
% G_MF_desejado(z) = N(z) / Hc(z)
tau_mf = 15;
d = 1;

Gs_MF_desejado = tf(1, [ tau_mf 1])
Gz_MF_desejado = c2d(Gs_MF_desejado, Ts)
    % b0 = 1-exp(-Ts/tau_mf)
    % a1 = exp(-Ts/tau_mf)

% H(z) = Hc(z)*Ho(z)
    % Polinomio desejado em M.F: Ho(z) := A(z) -> quando os polos são estaveis
    % Polinomio de "observador": Hc(z)

% T(z) = t_off*Ho(z)
    % t_off = Ho(z->1)/B(z->1) = A(1)/B(1)
    % t_off = (1 + a1) / b0;

% Eq. Diofantina
    % A(z)*R(z) + B(z)+S(z)*z^{-d} = 1 + h1*z{-1} + h2*z{-1}
   

%% RST
% R(z)*u(k) = T(z)*ref(k) - S(z)*y(k)

tfinal = 20;                % tempo total da simulação (s)
N = round( tfinal/Ts );     % numero total de amostras

% Sinal de referencia
ref(1:10) = 0; ref(round(N/2):N) = 1; 

% for k = 2:N
% 
%     % Modelo linear
%     y(k) = 
% 
%     % Controle
%     e(k) = ref(k) - y(k);
% 
%     % u(k) =
% end














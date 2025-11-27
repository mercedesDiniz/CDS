%% Atividade 02 (Parte 1) - CDS
% Resolvar o problema de alocação e teste sobre a plata G(s), mostrando que uma escolha de tau_mf de projeto (diferente de 10)
% funciona com seu controle RST sintetizado;
clear all; close all; clc;

%% Planta
tau = 10;
Gs = tf(1, [ tau 1]) 

% Discretizando a planta
Ts = 1; % periodo de amostragem (s)
Gz = c2d(Gs, Ts) % G(z) = ( B(z)z{-1} )/A(z) = b0/( 1+a1*z{-1} )
    Az = Gz.den{1}; a1 = Az(2);     % a1 = exp(-Ts/tau)
    Bz = Gz.num{1}; b0 = Bz(2);     % b0 = 1-exp(-Ts/tau)
    

%% Método de controle RST baseado em Astrom e Wittenmark (2011)
% R(z)*u(k) = T(z)*ref(k) - S(z)*y(k)

% Polinômio de atribuição de polos: H(z) = Hc(z)*Ho(z)

    % Polinomio de "observador": Ho(z) := A(z) -> quando os polos são estaveis
    Ho = Az;

    % Polinomio desejado em M.F: Hc(z)
    tau_mf = 15; % constante de tempo de malha fechada
    Gdes = tf(1,[tau_mf  1]);
    Gdesz = c2d(Gdes,Ts);
    Hc = Gdesz.den{1};

    H = conv(Hc,Ho);
        h1 = H(2); h2 = H(3);

% Polinômio R(z)
    r0 = 1;
    r1 = -1;

% Polinômio S(z)
    s0 = (h1 -a1+1)/b0;
    s1 = (h2 +a1)/b0;

% Polinômio T(z)
    % T(z) = t_off*Ho(z)
    % t_off = Hc(z->1)/B(z->1)
    t_off = sum(Hc)/sum(Bz); % pré-compensador DC
    Tz = t_off*Ho; 
        t0 = Tz(1); t1 = Tz(2);

%% Simulação no domínio do tempo discreto
    tfinal = 100;            % tempo total da simulação (s)
    N = round(tfinal/Ts);   % numero de amostras
    t = 0:Ts:N*Ts-Ts;       % vetor de tempo discreto
    
    % Sinal de referencia
    ref(1:N) = 0; ref(3:N) = 1; 
    
    % Condições iniciais
    y(1) = 0; u(1) = 0;

for k = 2:N
    y(k) = -a1*y(k-1) +b0*u(k-1);

    % Controlador RST
    u(k) = u(k-1) +t0*ref(k) +t1*ref(k-1) -s0*y(k) -s1*y(k-1);
end

% Plot
subplot(211)
    plot(t,ref,':k',t,y,'b');
    ylabel('Sinal de saída');
    legend('Ref.','y');
subplot(212)
    plot(t,u,'b');
    ylabel('Sinal de controle');

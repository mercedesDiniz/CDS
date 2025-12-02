%% Atividade 02 (Parte 2) - CDS
% Complemente a primeira parte da atividade 2 com a implementação do RST incremental aplicado
% no mesmo processo de 1ª ordem, mas com a imposição de uma dinâmica subamortecida em que pode-se
% selecionar o fator de amortecimento e frequencia natural desejados em malha fechada.
clear all; close all; clc;

%% Planta
tau = 10;
    Gs = tf(1, [ tau 1])  % G(s) = 1 / (10s+1)

    % Modelo discreto da planta
    Ts = 1; % periodo de amostragem (s)
    Gz = c2d(Gs, Ts)                   
        Az = Gz.den{1}; a1 = Az(2);
        Bz = Gz.num{1}; b0 = Bz(2);
    
    % Modelo aumentado da planta
    % y(k) = ( B(z) / ΔA(k) ) * Δu(k)
    % ΔA(k) = ( 1 - z^{-1} ) * ( 1 - a1*z^{-1} )
    delta = [1 -1]; % Δ = 1 - z^{-1} 
    delta_Az = conv(Az, delta);
        a1_aum = delta_Az(2); a2_aum = delta_Az(3);     
          
%% Projeto do controlador RST incremental
% R(z)*Δu(k) = T(z)*ref(k) - S(z)*y(k)

% Polinômio de atribuição de polos: H(z) = Hc(z)*Ho(z)

    % Polinomio de "observador": Ho(z) := A(z) -> quando os polos são estaveis
    Ho = Az;

    % Polinomio desejado em M.F: Hc(z)
        % Gs_des = ( g*ωn^{2} ) / ( s^{2}+2ζωns+ ωn^{2} )
        g = 1;         % ganho do sistema
        wn = 0.5;        % frequencia natural
        zeta = 0.5;    % fator de amortecimento (subamortecido: 0<ζ<1)

    Gs_des = tf(g*wn^2, [1  2*zeta*wn   wn^2])
    Gz_des = c2d(Gs_des,Ts)

    Hc = Gz_des.den{1};
    Hz = conv(Hc,Ho);
        % H(z) = 1 + h1*z^{-1} + h2*z^{-2} + h3*z^{-3}
        h1 = Hz(2); h2 = Hz(3); h3 = Hz(4);

    % Polinômio R(z)
        r0 = 1;
        r1 = 0;
    
    % Polinômio S(z)
        s0 = (h1 - a1_aum)/b0;
        s1 = (h2 - a2_aum)/b0;
        s2 = (h3)/b0;
    
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
    y(1:N) = 0; u(1:N) = 0;

for k = 3:N
    % Modelo da planta
    y(k) = -a1*y(k-1) +b0*u(k-1);

    % Controlador RST
    u(k) = u(k-1)+t0*ref(k)+t1*ref(k-1)-s0*y(k)-s1*y(k-1)-s2*y(k-2);
end

% Plot
subplot(211)
    plot(t,ref,':k',t,y,'b');
    ylabel('Sinal de saída');
    legend('Ref.','y');
subplot(212)
    plot(t,u,'b');
    ylabel('Sinal de controle');

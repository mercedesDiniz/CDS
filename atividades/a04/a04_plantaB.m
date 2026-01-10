%% Atividade 04: 
% Para as classes de plantas mostradas a seguir, projetar o sistemas GMVC incrementais de ordem mínima,
% realizando os seguintes testes e comparando-os com uma solução baseada em PID: 
clear all; close all; clc;

%% Planta b) G(s) = ( 4/(s^{2}+2s+4) )*e^{-0.3s}
Ts = 0.1; % periodo de amostragem (s)
d = round((0.3/Ts) + 1);   % atraso discreto (em amostras)
lambida = 1e-5;

    g = 1;          % ganho do sistema
    wn = 2;         % frequencia natural (rad/s)
    zeta = 0.5;     % fator de amortecimento

Gs = tf(g*wn^2, [1  2*zeta*wn   wn^2])
Gz = c2d(Gs,Ts, 'zoh')

Az = Gz.den{1};
    a0 = Az(1); a1 = Az(2); a2 = Az(3);
Bz = Gz.num{1};
    b0 = Bz(2); b1 = Bz(3);

    % Ampliar o modelo para incluir a diferença discreta.
    % Δ = 1-z^-1
    DAz = conv(Az,[1 -1]);
        da0 = DAz(1); da1 = DAz(2); da2 = DAz(3); da3 = DAz(4);

    % Equação Diofantina
        % E(z)= e0 + e1*z^{-1} + e2*z^{-2} + e3*z^{-3}
        % F(z)= f0 + f1*z^{-1} + f2*z^{-2}
    e0 = 1;
    e1 = -da1;
    e2 = -da2-da1*e1;
    e3 = -da3-da2*e1-da1*e2;
    f0 = -da3*e1-da2*e2-da1*e3;
    f1 = -da3*e2-da2*e3;
    f2 = -da3*e3;
    
    % R(z)
    r0 = b0*e0+lambida;
    r1 = b0*e1+b1*e0;
    r2 = b0*e2+b1*e1;
    r3 = b0*e3+b1*e2;
    r4 = b1*e3;

%% Simulação no dominio do tempo
tfinal = 100; % tempo de simulação (em segundos)
N = round( tfinal/Ts );
t = 0:Ts:N*Ts-Ts;

    % Sinal de referencia
    % ref(1:N) = 0; ref(3:N) = 1;
    ref(1:5)=0;
    ref(6: round( N/3 ))=1;
    ref(1+round( N/3 ): round(2*N/3) )=2.5;
    ref(1+round( 2*N/3 ): N )= 4;

    % Ruído de processo
    w = 0*wgn(1,N,1e-4,'linear');

    % Condições iniciais
    y(1:5) = 0; u(1:5) = 0; du(1:5) = 0;

for k = 6:N
    % Modelo da planta
    y(k) = (1/da0)*( -da1*y(k-1)-da2*y(k-2)-da3*y(k-3)+b0*du(k-4)+b1*du(k-5)+w(k) );

    % Controlador
    du(k) = (1/r0)*( -r1*du(k-1)-r2*du(k-2)-r3*du(k-3)-r4*du(k-4)+ref(k-4)-f0*y(k)-f1*y(k-1)-f2*y(k-2) ); 
    u(k) = u(k-1) +du(k);
end

%% Plot
figure;
subplot(211)
    plot(t,ref,':k',t,y,'b');
    ylabel('Sinal de saída');
    legend('Ref.','y');
subplot(212)
    plot(t,u,'b');
    ylabel('Sinal de controle');
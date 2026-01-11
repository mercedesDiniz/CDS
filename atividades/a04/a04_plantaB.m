%% Atividade 04: 
% Projetar o sistemas GMVC incrementais de ordem mínima 
clear all; close all; clc;

%% Planta b) G(s) = ( 4/(s^{2}+2s+4) )*e^{-0.3s}
Ts = 0.1; % periodo de amostragem (s)
d = round((0.3/Ts) + 1);   % atraso discreto (em amostras)
lambida = 10;

    ks = 1;         % ganho do sistema
    wn = 2;         % frequencia natural (rad/s)
    zeta = 0.5;     % fator de amortecimento

Gs = tf(ks*wn^2, [1  2*zeta*wn   wn^2])
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

    % Controlador PID
    kp = 0.1;  ki = 0; kd = 0.01;  
    s0 = kp +ki*Ts +kd/Ts;
    s1 = -kp -2*kd/Ts;
    s2 = kd/Ts;

%% Simulação no dominio do tempo
tfinal = 100; % tempo de simulação (em segundos)
N = round( tfinal/Ts );
t = 0:Ts:N*Ts-Ts;

    % Sinal de referencia
    ref(1:N) = 0; ref(3:N) = 1;
    % ref(1:5)=0;
    % ref(6: round( N/3 ))=1;
    % ref(1+round( N/3 ): round(2*N/3) )=2.5;
    % ref(1+round( 2*N/3 ): N )= 4;

    % Perturbação de carga
    v1 = zeros(1,N); v1(round(N/3):round(2*N/3)) = 0*0.5;

    % Ruído sensor
    v2 = 0*wgn(1,N,1e-4,'linear');

    % Ruído de processo
    w = 0*wgn(1,N,1e-6,'linear');

    % Condições iniciais
    y(1:5) = 0; u(1:5) = 0; du(1:5) = 0; dw(1:5) = 0;
    y_pid(1:5) = 0; u_pid(1:5) = 0; e(1:5) = 0;

for k = 6:N

    % Variação do ruído de processo
    dw(k) = w(k) - w(k-1);

    % Modelo da planta
    y(k) = (1/da0)*( -da1*y(k-1)-da2*y(k-2)-da3*y(k-3)+b0*du(k-4)+b1*du(k-5)+dw(k) ...
                     +v1(k)+da1*v1(k-1)+da2*v1(k-2)+da3*v1(k-3) ...
                     +v2(k)+da1*v2(k-1)+da2*v2(k-2) );

    y_pid(k) = (1/da0)*( -da1*y_pid(k-1)-da2*y_pid(k-2)-da3*y_pid(k-3)+b0*u_pid(k-4)+b1*u_pid(k-5)+dw(k) ...
                     +v1(k)+da1*v1(k-1)+da2*v1(k-2)+da3*v1(k-3) ...
                     +v2(k)+da1*v2(k-1)+da2*v2(k-2) );

    % Controlador GMVC incremental de ordem mínima
    du(k) = (1/r0)*( -r1*du(k-1)-r2*du(k-2)-r3*du(k-3)-r4*du(k-4)+ref(k-4)-f0*y(k)-f1*y(k-1)-f2*y(k-2) ); 
    u(k) = u(k-1) +du(k);

    % Cotrolador PID
    e(k) = ref(k) - y_pid(k);
    u_pid(k) = u_pid(k-1) + s0*e(k) +s1*e(k-1) + s2*e(k-2);
end

% Plot
figure;
subplot(211)
    plot(t,ref,':k',t,y,'b',t,y_pid,'r');
    ylabel('Sinal de saída');
    legend('Ref.','y_{GMVC}', 'y_{PID}');
subplot(212)
    plot(t,u,'b',t,u_pid,'r');
    legend('u_{GMVC}', 'u_{PID}');
    ylabel('Sinal de controle');

%% Análise dos índices ISE e ISU

    % Integral of Squared Error (ISE)
        % ISE pequeno = controle agressivo
    ISE = sum((ref - y).^2)
    
    % Integral of Squared Control Increment (ISU)
        % ISU pequeno = controle conservador
    ISU = sum(du.^2)

%% Análise das variâncias
    
    % Variância da saida
    var_y = var(y)
    % Variância do sinal de controle
    var_u = var(u)
    % Variância do erro
    var_e = var((ref - y))

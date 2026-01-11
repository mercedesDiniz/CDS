%% Atividade 04: 
% Projetar o sistemas GMVC incrementais de ordem mínima
clear all; close all; clc;

%% Planta a) G(s) = ( 1/(s+1) )*e^{-3s}
Ts = 1;   % periodo de amostragem (s)
d = round((3/Ts) + 1);   % atraso discreto (em amostras)
lambida = 1;

Gs = tf(1, [1 1])
Gz = c2d(Gs,Ts, 'zoh')

Az = Gz.den{1};
    a0 = Az(1); a1 = Az(2);
Bz = Gz.num{1};
    b0 = Bz(2);

    % Ampliar o modelo para incluir a diferença discreta.
    % Δ = 1-z^-1
    DAz = conv(Az,[1 -1]);
        da0 = DAz(1); da1 = DAz(2); da2 = DAz(3);

    % Equação Diofantina
        % E(z)= e0 + e1*z^{-1} + e2*z^{-2}
        % F(z)= f0 + f1*z^{-1}
    e0 = 1;
    e1 = -da1;
    e2 = -da1*e1-da2;
    e3 = -da1*e2-da2*e1;
    f0 = -da2*e2-da1*e3;
    f1 = -da2*e3;
    
    % R(z)
    r0 = b0*e0+lambida;
    r1 = b0*e1;
    r2 = b0*e2;
    r3 = b0*e3;

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
    w = 0*wgn(1,N,1e-4,'linear');
    
    % Condições iniciais
    y(1:4) = 0; dw(1:5) = 0; u(1:4) = 0; du(1:4) = 0;
    y_pid(1:4) = 0; u_pid(1:4) = 0; e(1:4) = 0;
     
for k = 5:N

    % Variação do ruído de processo
    dw(k) = w(k) - w(k-1);

    % Modelo da planta
    y(k) = (1/da0)*( -da1*y(k-1)-da2*y(k-2)+b0*du(k-4)+dw(k) ...
                     +v1(k)+da1*v1(k-1)+da2*v1(k-2) ...
                     +v2(k)+da1*v2(k-1)+da2*v2(k-2) );

    y_pid(k) = (1/da0)*( -da1*y_pid(k-1)-da2*y_pid(k-2)+b0*u_pid(k-4)+dw(k) ...
                     +v1(k)+da1*v1(k-1)+da2*v1(k-2) ...
                     +v2(k)+da1*v2(k-1)+da2*v2(k-2) );

    % Controlador GMVC incremental de ordem mínima
    du(k) = (1/r0)*( -r1*du(k-1)-r2*du(k-2)-r3*du(k-3)+ref(k-4)-f0*y(k)-f1*y(k-1) ); 
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

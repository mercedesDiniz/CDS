%% CDS - Atividade 01: Contole PID para o pêndulo amortecido
clear all; close all; clc;

%% Modelagem da planta do pêndulo amortecido

% Parametros 
J = 0.4;  % [kg*m^2]
c = 0.2;  % damping factor
m = 0.4;  % [kg]
g = 9.8;  % [m/s^2]
d = 0.05; % [m]
r = 0.4;  % [m]

Ts = 0.001; % em s

% Modelo contínuo linearizado
Gs = tf(r, [J  c  m*g*d]);  % (sin(θ)≈θ)

% Modelo discreto linearizado (discretizado por ZOH)
Gz = c2d(Gs,Ts,'zoh');
   Bz = Gz.num{1};
        b0 = Bz(2); b1 = Bz(3);
   Az = Gz.den{1};
        a1 = Az(2); a2 = Az(3);

%% Simulação da modelagem da planta
    % y_l x1_f(k) - posição do angulo (rad)
    % x2_f(k) - velocidade angular (rad/s)
    % u - força do sistema de propulsão (N)

tfinal = 20;                % tempo total da simulação (s)
N = round( tfinal/Ts );     % numero total de amostras

    % Condições iniciais
    x1_f(1:2)=0; x2_f(1:2)=0;
    y_l(1:2)=0;
    u(1:2)=0;

    for k = 3:N
        % Modelo linear (sin(θ)≈θ) discretizado por ZOH
        y_l(k) = -a1*y_l(k-1)-a2*y_l(k-2)+b0*u(k-1)+b1*u(k-2);

        % Modelo não linear discretizado de Forward
        x1_f(k) = x1_f(k-1) +Ts*x2_f(k-1);
        x2_f(k) = (1- c*Ts/J)*x2_f(k-1) -(m*g*d*Ts/J)*sin(x1_f(k-1)) +(r*Ts/J)*u(k-1);

        u(k) = 0.25; % controle em malha aberta
    end

% Plot
t = 0:Ts:N*Ts-Ts;
subplot(211)
    plot(t,x1_f,'b',t,y_l,'r');
    ylabel('Posição angular (rad)');
    legend('Não linear','Linear');
subplot(212)
    plot(t,(180/pi)*x1_f,'b',t,(180/pi)*y_l,'r');
    ylabel('Posição angular (deg)');
    legend('Não linear','Linear');

%% Controlador PID
kp = 1;  ki = 0.4; kd = 0.6; % ganhos
   
    % PID digital baseado na aproximação de Backward diff
    s0 = kp +ki*Ts +kd/Ts;
    s1 = -kp -2*kd/Ts;
    s2 = kd/Ts;
    
%% Analise de margens de ganho e de fase
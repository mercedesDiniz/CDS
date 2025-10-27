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

% Modelo contínuo linearizado (sin(θ)≈θ)
    %                           r
    %  theta(s)/F(s) = -------------------
    %                   J*s^2 +c*s +m*g*d

Gs_l = tf(r, [J  c  m*g*d])

% Modelo discreto linearizado (discretizado por ZOH)
Gz_l = c2d(Gs_l,Ts,'zoh');
   Bz = Gz_l.num{1};
        b0_l = Bz(2); b1_l = Bz(3);
   Az = Gz_l.den{1};
        a1_l = Az(2); a2_L = Az(3);

% Modelo contínuo não-linear
    %                               r
    %  theta(s)/F(s) = ------------------------------
    %                   J*s^2 +c*s +m*g*d*sin(theta)


% Modelo discreto não-linear (discretizado por ZOH)


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
        y_l(k) = -a1_l*y_l(k-1)-a2_L*y_l(k-2)+b0_l*u(k-1)+b1_l*u(k-2);

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
% Projeto baseado no modelo linear
kp = 1;  ki = 0.4; kd = 0.6; % ganhos
   
    % PID digital baseado na aproximação de Backward diff
    s0 = kp +ki*Ts +kd/Ts;
    s1 = -kp -2*kd/Ts;
    s2 = kd/Ts;

    % Condições iniciais
    x1_f(1:2)=0; x2_f(1:2)=0;
    u_f(1:2)=0; e_f(1:2)=0;

    y_l(1:2)=0;
    u_l(1:2)=0; e_l(1:2)=0;

    % Sinal de referencia
    ref(1:10) = 0; ref(11:N) = 5*(pi/180); % rad

    for k = 3:N

        % Modelo linear de tempo continuo (ZOH)
        y_l(k) = -a1_l*y_l(k-1)-a2_L*y_l(k-2)+b0_l*u(k-1)+b1_l*u(k-2);

        % Modelo não linear de tempo continuo (ZOH)

        % Modelo não linear de tempo discreto (Forward)
        x1_f(k) = x1_f(k-1) +Ts*x2_f(k-1);
        x2_f(k) = (1- c*Ts/J)*x2_f(k-1) -(m*g*d*Ts/J)*sin(x1_f(k-1)) +(r*Ts/J)*u(k-1);
        
    end
        
%% Analise de margens de ganho e de fase
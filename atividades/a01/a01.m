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

Gs_l = tf(r, [J  c  m*g*d]);

% Modelo discreto linearizado (discretizado por ZOH)
Gz_l = c2d(Gs_l,Ts,'zoh');
   Bz = Gz_l.num{1};
        b0 = Bz(2); b1 = Bz(3);
   Az = Gz_l.den{1};
        a1 = Az(2); a2 = Az(3);

    % y_l(k) = -a1_l*y_l(k-1)-a2_L*y_l(k-2)+b0_l*u(k-1)+b1_l*u(k-2);

% Modelo discreto não-linear (discretizado por Forward)
    % x1_nl(k) = x1_nl(k-1) +Ts*x2_nl(k-1);
    % x2_nl(k) = (1- c*Ts/J)*x2_nl(k-1) -(m*g*d*Ts/J)*sin(x1_nl(k-1)) +(r*Ts/J)*u_nl(k-1);


%% Simulação da modelagem da planta
    % y_l x1_f(k) - posição do angulo (rad)
    % x2_f(k) - velocidade angular (rad/s)
    % u - força do sistema de propulsão (N)

tfinal = 25;                % tempo total da simulação (s)
N = round( tfinal/Ts );     % numero total de amostras

    % Condições iniciais
    x1_nl(1:2)=0; x2_nl(1:2)=0;
    y_l(1:2)=0;
    u(1:2)=0;

    for k = 3:N
        % Modelo linear (sin(θ)≈θ) discretizado por ZOH
        y_l(k) = -a1*y_l(k-1)-a2*y_l(k-2)+b0*u(k-1)+b1*u(k-2);
                  

        % Modelo não linear discretizado de Forward
        x1_nl(k) = x1_nl(k-1) +Ts*x2_nl(k-1);
        x2_nl(k) = (1- c*Ts/J)*x2_nl(k-1) -(m*g*d*Ts/J)*sin(x1_nl(k-1)) +(r*Ts/J)*u(k-1);

        u(k) = 0.25; % controle em malha aberta
    end

% Plot
t = 0:Ts:N*Ts-Ts;
figure;
subplot(211)
    plot(t,x1_nl,'b',t,y_l,'r');
    ylabel('Posição angular (rad)');
    legend('Não linear','Linear');
subplot(212)
    plot(t,(180/pi)*x1_nl,'b',t,(180/pi)*y_l,'r');
    ylabel('Posição angular (deg)');
    legend('Não linear','Linear');

%% Controlador PID
% Projeto baseado no modelo linear
kp = 1;  ki = 0.4; kd = 0.6; % ganhos
   
    % PID digital baseado na aproximação de Backward diff
    s0 = kp +ki*Ts +kd/Ts;
    s1 = -kp -2*kd/Ts;
    s2 = kd/Ts;

    % Perturbação de carga
    v1(1:N/2) = 0; v1(1+(N/2):N) = 0.5*(pi/180); % rad

    % Ruido gaussiano
    v2 = 0*wgn(1,N,1e-4,'linear'); % W

    % Condições iniciais
    x1_nl(1:2)=0; x2_nl(1:2)=0;
    u_nl(1:2)=0; e_nl(1:2)=0;

    y_l(1:2)=0;
    u_l(1:2)=0; e_l(1:2)=0;

    % Sinal de referencia
    ref(1:10) = 0; ref(11:N) = 5*(pi/180); % rad

    for k = 3:N

        % Modelo linear
        y_l(k) = -a1*y_l(k-1)-a2*y_l(k-2)+b0*u(k-1)+b1*u_l(k-2) ...
                    + v1(k) +a1*v1(k-1) +a2*v1(k-2) ...    
                    + v2(k) +a1*v2(k-1) +a2*v2(k-2);

            % Controle
            e_l(k) = ref(k) - y_l(k);
            u_l(k) = u_l(k-1) + s0*e_l(k) +s1*e_l(k-1) + s2*e_l(k-2);

        % Modelo não linear (Forward)
        x1_nl(k) = x1_nl(k-1) +Ts*x2_nl(k-1);
        x2_nl(k) = (1- c*Ts/J)*x2_nl(k-1) -(m*g*d*Ts/J)*sin(x1_nl(k-1)) +(r*Ts/J)*u_nl(k-1);
        y_nl(k) = x1_nl(k) + v1(k) + v1(k);

            % Controle
            e_nl = ref(k) - x1_nl;
            u_nl(k) = u_nl(k-1) + s0*e_nl(k) + s1*e_nl(k-1) + s2*e_nl(k-2);
    end

% Plots
t = 0:Ts:N*Ts-Ts;
figure;
subplot(311)
    plot(t,ref,'k',t,y_nl,'b',t,y_l,'r');
    ylabel('Posição angular (rad)');
    legend('Ref.','Non linear','Linear');
subplot(312)
    plot(t,(180/pi)*ref,'k',t,(180/pi)*y_nl,'b',t,(180/pi)*y_l,'r');
    ylabel('Posição angular (deg)');
    legend('Ref.','Non linear','Linear');
subplot(313)
    plot(t,u_nl,'b', t,u_l,'--r');
    ylabel('F(t) (N)');
    legend('Non linear','Linear');

%% Analise de margens de ganho e de fase

% Control system relative stability analysis
Cz = tf([s0 s1 s2],[1 -1 0],Ts);

Gdlz = Cz*Gz_l; % Direct loop system
w = logspace(-2,5,100);
figure; margin(Gdlz,w); % Gain and Phase Margins


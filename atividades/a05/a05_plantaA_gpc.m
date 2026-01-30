%% Atividade 05: Projetar e implementar, por simulação, o  PMVC, DMC e GPC.
clear all; close all; clc;

%% Parametros
Ts = 0.1;                % periodo de amostragem (s)
td = 0;                  % atraso continuo (s)
d = round((td/Ts) + 1)   % atraso discreto (em amostras)

Ny = 2; Nu = 1;          % horizonte de predição
lam = .000001;           % fator de ponderação de controle

%% Planta a) Dinâmica do ângulo de roll do VLS-1 em Max Q

% Modelo contínuo
    Gs = tf(61.0492, [1  0.098859 0]);
    
    % Verificando os autovalores
    disp('Autovalores da planta:')
    eig(Gs)

% Modelo discreto
    Gz = c2d(Gs,Ts, 'zoh')
        Az = Gz.den{1};
            a1 = Az(2); a2 = Az(3);
        Bz = Gz.num{1}
            b0 = Bz(2); b1 = Bz(3);

% Converting the system to the ARIMAX state-space form:
DAz = conv(Az,[1 -1])
A = [ -DAz(2:end)'  [ eye(2,2) ; [0 0] ]   ]
B = [ Bz(2:end)' ; 0]
C = [ 1 0 0 ];
Gamma = [ -DAz(2:end)' ]
% sysa = ss(A,B,C,0,Ts); figure; step(sysa)


%% GPC
nb=1; na = 2;

    Eorig =[1]; 
    E=1; F=[A^0*Gamma]'; Gmat=[];
    Gj=conv(E,Bz(2:end)); Gbar=[Gj(end-nb+1:end)];

    for j=2:Ny
        E=[E C*A^(j-2)*Gamma]; F=[F;[A^(j-1)*Gamma]'];
        Gj=conv(E,Bz(2:end)); Gbar=[Gbar; Gj(end-nb+1:end)];
    end

% Matriz G
    G = toeplitz( Gj(1:end-nb) ); G=tril(G);
    G = G(1:Ny,1:Nu); 

% Matriz F_R 
    F_R = [Gbar F];

% Ganho
    K = inv(G'*G +lam*eye(Nu,Nu))*G';
    K1 = K(1,:);

%% Simulação
tfinal = 10; 
Nsim = round(tfinal/Ts);

% Sinal de referencia
r(1:na+d)=0; r(na+d+1:Nsim+Ny)=1;
r = r'; 

for k = 1:na+d % initial conditions (N is huge!)
    y(k)=0;
    u(k)=0;
    du(k)=0;
end
for k = na+d+1:Nsim
    % Planta
    y(k) = -a1*y(k-1) -a2*y(k-2) +b0*u(k-d) +b1*u(k-d-1);

    % Controlador
    x = [du(k-1) y(k) y(k-1) y(k-2)]';

    du(k) = K1*( r(k:k+Ny-1,1)-F_R*x );

    u(k) = u(k-1) +du(k);

    % Saturação
    % if u(k) >= 0.06981
    %   u(k) = 0.06981;
    % elseif u(k) <= 0
    %   u(k) = 0;
    % end
end

%% Plots
t=0:Ts:Nsim*Ts-Ts;
figure;
subplot(211)
    plot(t,r(1:Nsim),':k'); hold;
    plot(t,y,'b');

subplot(212)
    plot(t,u,'b');
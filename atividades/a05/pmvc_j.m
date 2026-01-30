%% PMVC - Atividade 5
clear all; close all; clc;


%% Planta 1
% Modelo de Espaço de estados

Ac = [0              1  ;
      0  -0.098859];
Bc = [0;  61.0492];
C = [1 0];
D = 0;
sysc = ss(Ac,Bc,C,D);

% Equivalente ZOH - Modelo de Espaço de estados 
tau = 1/0.098859

Ts = tau/50

%Ts = 1; % approx.
sysd = c2d(sysc,Ts);
[Ad,Bd,Cd,Dd] = ssdata(sysd)

td = 0 % Segundo o comando da atividade, considerar o atraso igual a 0
   

% Total discrete time delay
d = 1+ round(td/Ts);
disp('Atraso do tempo discreto '); disp(d);


% Modelo aumentado

Aa = [ 1          Cd*Ad;
      zeros(2,1)  Ad  ];
Ba = [Cd*Bd  ;  Bd];
Ca = [Cd 0];
Da = 0;
sysa = ss(Aa,Ba,Ca,Da,Ts)


% Filtro de Kalman baseado no estimador
    Qkf = Ca'*Ca;
    Rkf = 1;
   % L = [ dlqr(Aa',Ca',Qkf,Rkf) ]'

Po = 1e2*eye(3,3); % Ordem da Matriz A
    for i= 1:100
        Po = Aa*Po*Aa'-Aa*Po*Ca'*inv(Ca*Po*Ca'+Rkf)*Ca*Po*Aa'+Qkf; 
        tracePo(i)=trace(Po);
    end 

    figure (1); 
    plot(tracePo);

    L = (Aa*Po*Ca'*inv(Ca*Po*Ca'+Rkf))
    disp('Verificando os autovalores do observador de estado em malha fechada');
    eig(Aa-L*Ca)

% Verificando a convergência do estimador baseado em Kalman
    syskf = ss(Aa-L*Ca, L, Ca, Da, Ts);
    figure(2); 
    step(syskf);
    title('Análise de convergência do estimador baseado em Kalman');


% Ganho LQR 
    Qlq = diag([.00001   1   1e-4]);
    Rlq = 10000;
    K =  dlqr(Aa,Ba,Qlq,Rlq) 
    disp('Verificando os autovalores de malha fechada do LQR');
    eig(Aa-Ba*K)

% Verificação do servocontrol LQR
    syslq = ss(Aa-Ba*K, Ba*K, Ca, Da, Ts);
    figure(3); 
    step(syslq);
    title('Análise de servocontrole do LQR');

% PMVC
Nx = d+1; %Horizonte de predição
LAM = diag([125]); % Fator de ponderação

    % Matrizes principais
    Mx = K*( Aa^Nx -(Aa^(Nx-1))*L*Ca );
    
    Mu = [];
    for i = 1:Nx-1
	    Mu = [Mu (Aa^i)*Ba];
    end
    
    Mu = K*Mu
    My = K*Aa^(Nx-1)*L


% Simulação discreta do sistema com PMVC 
tfinal = 220; % total time simulation
N = round( tfinal/Ts ); % total number of samples

% Reference sequence
r1(1:Nx+1) = 0; r1(Nx+2:N) = 1; % for y1
    

for k = 1:Nx 
    % Plant model
    x(:,:,k)=[0;0];
    u(k) = 0;
    y(k) = Cd*x(:,:,k);

    % Controller
    xa(:,:,k) = zeros(3,1);
    du(k) = 0;
end

for k = Nx+1:N
    % Plant model
    x(:,:,k) = Ad*x(:,:,k-1) +Bd*u(k-d);
    y(k) = Cd*x(:,:,k);


    %PMVC
    xa(:,:,k) = (Aa-L*Ca)*xa(:,:,k-1) +Ba*du(k-Nx) +L*y(k-1);
    
        % DU vector from Eq. (18)
        DU = [];
        for i = 1:Nx-1
            DU = [DU; du(k-i) ];
        end

    du(k) = inv(K*Ba+LAM)*( K*[r1(k);0;0] -Mx*xa(:,:,k) ...
        -Mu*DU -My*y(k) );

    % Testing the LQG instead of PMVC
    %xa(:,:,k) = (Aa-L*Ca)*xa(:,:,k-1) +Ba*du(k-1) +L*y(k-1);
    %du(k) = K*([r1(k);0;0] -xa(:,:,k));

    % Control increment

    u(k) = u(k-1) +du(k);

  if u(k) >= 0.06981
      u(k) = 0.06981;
  elseif u(k) <= 0
      u(k) = 0;
  end

end

%% Plots
t = 0:Ts:N*Ts-Ts;


figure(4);

subplot(311)
plot(t,r1,':k'); hold on;
plot(t,y,'b');
ylabel('y(t)');
xlabel('Time (s)');
grid on
legend('Reference','Ângulo (rad)')

subplot(312)
plot(t,(180/pi)*y,'r');
ylabel('Ângulo (deg)');
legend('y(t)')
grid on

subplot(313)
plot(t,du,'b');
ylabel('\Delta u(t)');
xlabel('Time (s)');
grid on
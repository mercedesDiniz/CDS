%% Atividade 05: Projetar e implementar, por simulação, o  PMVC, DMC e GPC.
clear all; close all; clc;

%% Parametros
Ts = 0.1;                % periodo de amostragem (s)
td = 0;                  % atraso continuo (s)
d = round((td/Ts) + 1)   % atraso discreto (em amostras)

Nx = d+1;                % horizonte de previsão de estado (atraso virtual)

%% Planta a) Dinâmica do ângulo de roll do VLS-1 em Max Q

% Modelo em espaço de estados contínuo
    A = [0      1;
         0  -0.098859];
    B = [0; 1];
    
    C = [61.0492    0];
    
    D = 0;
    
    sysc = ss(A,B,C,D)

% Modelo em espaço de estados discreto
    sysd = c2d(sysc,Ts);
    
    [Ad,Bd,Cd,Dd] = ssdata(sysd);

% Modelo aumentado em espaço de estados
    Aa = [ 1          Cd*Ad;
          zeros(2,1)  Ad  ];
    
    Ba = [Cd*Bd  ;  Bd];
    
    Ca = [Cd 0];
    
    Da = 0;
    
    sysa = ss(Aa,Ba,Ca,Da,Ts);

%% Testes de Controlabilidade e Observabilidade
n = length(Aa);     % Número de variaveis de estado (o integrador é considerado)
Co = ctrb(Aa, Ba);  % Matriz de Controlabilidade
Ob = obsv(Aa, Ca);  % Matriz de Observabilidade

disp('Verificação da Controlabilidade:');
    disp(['> rank(Co) = ' num2str(rank(Co))]);
    if rank(Co) == n
        disp(['== ' num2str(n) ' - O sistema é controlável' ]);
    else
        disp(['!= ' num2str(n) ' - O sistema não é controlável' ]);
    end

disp('Verificação da Observabilidade:');
    disp(['> rank(Ob) = ' num2str(rank(Ob))]);
    if rank(Ob) == n
        disp(['== ' num2str(n) ' - O sistema é observável' ]);
    else
        disp(['!= ' num2str(n) ' - O sistema não é observável' ]);
    end

%% PMVC com realimentação total de estados

%% Estimador de estados baseado no filtro de Kalman
    Qkf = Ca'*Ca;  % covariancia do ruido do processo
    Rkf = 1e10;             % covariancia do ruido de medição
    
    disp('Verificação da Detectabilidade (p/ o Filtro de Kalman):');
    Ob_dec_kf = obsv(Aa', sqrt(Qkf)');
    disp(['> rank(obsv(Aa^(T), sqrt(Qkf)^(T))) = ' num2str(rank(Ob_dec_kf))]);
    
    if rank(Ob_dec_kf) == n
        disp(['== ' num2str(n) ' - O sistema é detectável' ]);
    else
        disp(['!= ' num2str(n) ' - O sistema não é detectável' ]);
    end

    % Calculo da matriz P pela Equação a Diferenças de Riccati
    Po = 1e6*eye(n, n); p_samples = 1000; 
    for i = 1:p_samples
        Po = Aa*Po*Aa' -Aa*Po*Ca'*inv(Ca*Po*Ca'+Rkf)*Ca*Po*Aa'+Qkf;
        tracePo(i)=trace(Po); 
    end
    % figure; plot(tracePo); ylabel('Traço de P - Filtros de Kalman'); xlabel('Amostras');

    % Calculo do Ganho
    L = ( Aa*Po*Ca'*inv(Ca*Po*Ca'+Rkf) );
    % L = [ dlqr(Aa',Ca',Qkf,Rkf) ]'

    % Verificando os autovalores
    disp('Autovalores do KF em MF:')
    eig(Aa-L*Ca)

    % Verificando se converge
    syskf = ss(Aa-L*Ca, L, Ca, Da, Ts);
    figure; step(syskf); title('Kalman-based estimator convergence analysis');

%% LQR
    Qlqr = diag([0   1   0]);  % penalização dos estados
    Rlqr = 1e4;                % penalização do esforço de controle

    disp('Verificação da Detectabilidade (p/ o LQR):');
    Ob_dec_lqr = obsv(Aa', sqrt(Qlqr)');
    disp(['> rank(obsv(Aa^(T), sqrt(Qlqr)^(T))) = ' num2str(rank(Ob_dec_lqr))]);
    
    if rank(Ob_dec_lqr) == n
        disp(['== ' num2str(n) ' - O sistema é detectável' ]);
    else
        disp(['!= ' num2str(n) ' - O sistema não é detectável' ]);
    end
    
    % Calculo da matriz P pela Equação a Diferenças de Riccati
    P = 1e6*eye(n, n); p_samples = 1000; 
    for i = 1:p_samples
        P = Aa'*P*Aa -Aa'*P*Ba*inv(Ba'*P*Ba+Rlqr)*Ba'*P*Aa+Qlqr;
        traceP(i)=trace(P); 
    end
    %figure; plot(traceP); ylabel('Traço de P - LQR'); xlabel('Amostras');

    % Calculo do Ganho
    K = ( Aa'*P*Ba*inv(Ba'*P*Ba+Rlqr) )';
    % K =  dlqr(Aa,Ba,Qlqr,Rlqr)

    % Verificando os autovalores
    disp('Autovalores do LQR em MF:')
    eig(Aa-Ba*K)

    % Verificando se converge
    syslq = ss(Aa-Ba*K, Ba*K(1), Ca, Da, Ts);
    figure; step(syslq); title('LQR servocontrol analysis');

    % Variaveis auxiliares
        % Fator de ponderação de controle
        LAM = diag([125]); %diag([.1]); 
    
        % Matrizes principais da leis de controle
        Mx = K*( Aa^Nx -(Aa^(Nx-1))*L*Ca );
        
        Mu = [];
        for i = 1:Nx-1
	        Mu = [Mu (Aa^i)*Ba];
        end
	        Mu = K*Mu;
        
        My = K*Aa^(Nx-1)*L;

%% Simulação
tfinal = 300;           % tempo de simulação (em segundos)
N = round( tfinal/Ts );
t = 0:Ts:N*Ts-Ts;      

    % Sinal de referencia
    r1(1:Nx+1) = 0; r1(Nx+2:N) = 1; 
    
    % Condições iniciais
    for k = 1:Nx 
        x(:,:,k)=[0;0];
        u(k) = 0;
        y(k) = Cd*x(:,:,k);

        xa(:,:,k) = zeros(3,1);
        du(k) = 0;
    end

for k = Nx+1:N
    % Planta
    x(:,:,k) = Ad*x(:,:,k-1) +Bd*u(k-d);
    y(k) = Cd*x(:,:,k);

    % Controlador
    xa(:,:,k) = (Aa-L*Ca)*xa(:,:,k-1) +Ba*du(k-Nx) +L*y(k-1);
    
        DU = [];
        for i = 1:Nx-1
            DU = [DU; du(k-i) ];
        end

    du(k) = inv(K*Ba+LAM)*( K*[r1(k);0;0] -Mx*xa(:,:,k) ...
        -Mu*DU -My*y(k) );

    u(k) = u(k-1) +du(k);

    % Saturação
    if u(k) >= 0.06981
      u(k) = 0.06981;
    elseif u(k) <= 0
      u(k) = 0;
    end

end

%% Plots
t = 0:Ts:N*Ts-Ts;
figure;
    subplot(211)
    plot(t,r1,':k'); hold;
    plot(t,y,'b');
    ylabel('y(t)'); xlabel('Time (s)');
    subplot(212)
    plot(t,du,'b');
    ylabel('du(t)'); xlabel('Time (s)');

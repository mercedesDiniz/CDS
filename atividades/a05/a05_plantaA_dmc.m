%% Atividade 05: Projetar e implementar, por simulação, o  PMVC, DMC e GPC.
clear all; close all; clc;

%% Parametros
Ts = 1;                 % periodo de amostragem (s)
td = 0;                 % atraso continuo (s)
d = round((td/Ts) + 1)  % atraso discreto (em amostras)

Ny = 3; Nu = 1;         % horizonte de predição
lam = 1e10;             % fator de ponderação de controle

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

%% DMC

% Resposta ao degral para criar o database
tfinal = 10; 
Ntest = round(tfinal/Ts); 
y = step(Gz,Ntest);
 %plot(y)

g = y(1:81); 
g = g(2:81);    % Remark: getting rid of the null initial value!
N = length(g); 
clear y;

% Matriz G
Gorig = toeplitz(g); % Toeplitz Matrix based on g
Gorig = tril(Gorig); % Lower Trianglar matrix based on Gorig

  G = Gorig(1:Ny,1:Nu)

% Ganho baseado em G e lam
  K = inv( G'*G +lam*eye(Nu,Nu) )*G'

  K1 = K(1,:)

% Matriz F
Fi = []; 
  for i = 1:N-Ny % REMARK!!!
      Fi = [Fi g(i+1:Ny+i,1)-g(i,1)];
  end
    F = [ones(Ny,1) Fi];
    
%% Simulação
tfinal = 1400; 
Nsim = round(tfinal/Ts); % total number of iterations for the sim.

r(1:N)=0; r(N+1:Nsim+Ny)=1;
r = r'; % Making sure that this is a column vector!

for k = 1:N % initial conditions (N is huge!)
    y(k)=0;
    u(k)=0;
    du(k)=0;
end
for k = N+1:Nsim
    % Plant model
    y(k) = -a1*y(k-1) -a2*y(k-2) +b0*u(k-d) +b1*u(k-d-1);

    % DMC
      U = []; % du(k) regressors
      for i=1:N-Ny
          U = [U; du(k-i)];
      end
      x = [y(k);U];

      % Control law
      du(k) = K1*( r(k:k+Ny-1,1)-F*x );

      u(k) = u(k-1) +du(k);
end

%% Plots
t=0:Ts:Nsim*Ts-Ts;
figure;
subplot(211)
    plot(t,r(1:Nsim),':k'); hold;
    plot(t,y,'b');

subplot(212)
    plot(t,u,'b');

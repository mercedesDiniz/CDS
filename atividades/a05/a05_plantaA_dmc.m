%% Atividade 05: Projetar e implementar, por simulação, o  PMVC, DMC e GPC.
clear all; close all; clc;

%% Parametros
Ts = 0.1;             % periodo de amostragem (s)
td = 0;                  % atraso continuo (s)
d = round((td/Ts) + 1)   % atraso discreto (em amostras)

% Nx = d+1;                % horizonte de previsão de estado (atraso virtual)

%lambida = 1;

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

%% Step-response data to build the DMC database
tfinal = 100; % total sim. time in seconds
Ntest = round(tfinal/Ts); % total number of samples to simulate
y = step(Gz,Ntest);
 %plot(y)

g = y(1:81); % Selected by visual inspection of the plot
g = g(2:81); % Remark: getting rid of the null initial value!
N = length(g); % total number of regressors for the free response model
clear y;

%% Creating a database matrix Goriginal from where we can
%% generate the G matrix.
Gorig = toeplitz(g); % Toeplitz Matrix based on g
Gorig = tril(Gorig); % Lower Trianglar matrix based on Gorig
  % Gorig(1:5,1:5)

  % Prediction Horinzons and control weighting factor
  Ny = 3; Nu = 1; lam = 1e10;
  G = Gorig(1:Ny,1:Nu)

  % DMC gain based on G and lam
  disp('Complet gain K = ');
  K = inv( G'*G +lam*eye(Nu,Nu) )*G'

  disp('Just the 1st line of K:');
  K1 = K(1,:)

  %% Generating the matrix F for the free response model
  Fi = []; % internal part of the F matrix

  %N = 5+Ny+d; % Remark: forcing a smaller N by pure testing

  for i = 1:N-Ny % REMARK!!!
      Fi = [Fi g(i+1:Ny+i,1)-g(i,1)];
  end
    F = [ones(Ny,1) Fi];
    


%% Discrete-time simulation
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

% Plots
t=0:Ts:Nsim*Ts-Ts;
figure;
subplot(211)
    plot(t,r(1:Nsim),':k'); hold;
    plot(t,y,'b');

subplot(212)
    plot(t,u,'b');



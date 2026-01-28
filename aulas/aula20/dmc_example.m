%% Dynamic Matrix Control Example
clear all; close all; clc;

%% Example model
ks = 1; % static gain
wn = 2; % rad/s
zeta = 0.3; % damping factor
td = 0; % time delay in seconds
Gs = tf(ks*wn^2,[1   2*zeta*wn  wn^2]);



%% ZOH equivalent
ws = 10*wn; % sampling frequency in rad/s
    fs = ws/(2*pi); % ... in Hz
    Ts = 1/fs
    Ts = 0.1; % approx.
Gz = c2d(Gs,Ts);
Az = Gz.den{1};
    a1 = Az(2); a2 = Az(3);
Bz = Gz.num{1}
    b0 = Bz(2); b1 = Bz(3);

    % Total discrete time delay
    d = 1+ round(td/Ts);
    disp('Discrete time delay d = '); disp(d);

    %Gz = tf(Bz,Az,Ts,'inputdelay',round(td/Ts))
    Gz = tf(Bz,Az,Ts);

%% Step-response data to build the DMC database
tfinal = 10; % total sim. time in seconds
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
  Ny = 2; Nu = 1; lam = 10;
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
tfinal = 140; 
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
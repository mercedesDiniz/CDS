%% Generalized Predictive Control Example
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
Bz = Gz.num{1};
    b0 = Bz(2); b1 = Bz(3);

    % Total discrete time delay
    d = 1+ round(td/Ts);
    disp('Discrete time delay d = '); disp(d);

    %Gz = tf(Bz,Az,Ts,'inputdelay',round(td/Ts))
    Gz = tf(Bz,Az,Ts);

%% Converting the system to the ARIMAX state-space form:
DAz = conv(Az,[1 -1])
A = [ -DAz(2:end)'  [ eye(2,2) ; [0 0] ]   ]
B = [ Bz(2:end)' ; 0]
C = [ 1 0 0 ];
Gamma = [ -DAz(2:end)' ]
% sysa = ss(A,B,C,0,Ts); figure; step(sysa)


%% GPC
nb=1; na = 2;
Ny = 2; Nu = 1; lam = .000001;

    % Generating a general Gorig matrix to obtain G and Gbar
    % Rmk: Gj(z) = B(z)Ej(z)
    Eorig =[1]; % based on Ej(z)
    E=1; F=[A^0*Gamma]'; Gmat=[];
    Gj=conv(E,Bz(2:end)); Gbar=[Gj(end-nb+1:end)];
    % Construindo as matrizes Gbar e F:
    for j=2:Ny
    E=[E C*A^(j-2)*Gamma]; F=[F;[A^(j-1)*Gamma]'];
    Gj=conv(E,Bz(2:end)); Gbar=[Gbar; Gj(end-nb+1:end)];
    end

    % G matrix obtained by the Toeplitz lower tringular matrix
    G = toeplitz( Gj(1:end-nb) ); G=tril(G);
    G = G(1:Ny,1:Nu); % Based on the prediction horizons

    % F_R matrix: Eq. (6.270)
    F_R = [Gbar F];

    % GPC Gain
    K = inv(G'*G +lam*eye(Nu,Nu))*G';
    K1 = K(1,:);

%% Discrete-time simulation
tfinal = 10; 
Nsim = round(tfinal/Ts); % total number of iterations for the sim.

r(1:na+d)=0; r(na+d+1:Nsim+Ny)=1;
r = r'; % Making sure that this is a column vector!

for k = 1:na+d % initial conditions (N is huge!)
    y(k)=0;
    u(k)=0;
    du(k)=0;
end
for k = na+d+1:Nsim
    % Plant model
    y(k) = -a1*y(k-1) -a2*y(k-2) +b0*u(k-d) +b1*u(k-d-1);

    % GPC
      x = [du(k-1) y(k) y(k-1) y(k-2)]';

      % Control law
      du(k) = K1*( r(k:k+Ny-1,1)-F_R*x );

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
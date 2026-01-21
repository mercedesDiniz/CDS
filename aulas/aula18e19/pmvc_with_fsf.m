%% PMVC with full state feedback
clear all; close all; clc;

%% Example model
ks = 1; % static gain
wn = 2; % rad/s
zeta = 0.3; % damping factor
td = 0.5; % time delay in seconds



%% State-space realization
Ac = [0              1  ;
      -wn^2  -2*zeta*wn];
Bc = [0;  ks*wn^2];
C = [1 0];
D = 0;
sysc = ss(Ac,Bc,C,D);

%% State-space model ZOH equivalent
ws = 10*wn; % sampling frequency in rad/s
    fs = ws/(2*pi); % ... in Hz
    Ts = 1/fs
    Ts = 0.1; % approx.
sysd = c2d(sysc,Ts);
[Ad,Bd,Cd,Dd] = ssdata(sysd)

    % Total discrete time delay
    d = 1+ round(td/Ts);
    disp('Discrete time delay d = '); disp(d);

%% Model augmentation to include integrator at the
% input.
Aa = [ 1          Cd*Ad;
      zeros(2,1)  Ad  ];
Ba = [Cd*Bd  ;  Bd];
Ca = [Cd 0];
Da = 0;
sysa = ss(Aa,Ba,Ca,Da,Ts);

%% Kalman filter based estimator
Qkf = Ca'*Ca;
Rkf = 1;
L = [ dlqr(Aa',Ca',Qkf,Rkf) ]'
disp('Checking the state observer closed-loop eigenvalues');
eig(Aa-L*Ca)

    % Checking the Kalman-based estimator convergence
    syskf = ss(Aa-L*Ca, L, Ca, Da, Ts);
    figure; step(syskf);
       title('Kalman-based estimator convergence analysis');

%% LQR gain
Qlq = diag([1  50  0]);
Rlq = 1;
K =  dlqr(Aa,Ba,Qlq,Rlq) 
disp('Checking the LQR closed-loop eigenvalues');
eig(Aa-Ba*K)
    % Checking the LQR servocontrol
        syslq = ss(Aa-Ba*K, Ba*K(1), Ca, Da, Ts);
        figure; step(syslq);
        title('LQR servocontrol analysis');


%% PMVC
Nx = d+1; % State-prediction horizon (virtual delay)
LAM = diag([.1]); % control weighting factor

    % Main control law matrices
    Mx = K*( Aa^Nx -(Aa^(Nx-1))*L*Ca );
    
    Mu = [];
    for i = 1:Nx-1
	    Mu = [Mu (Aa^i)*Ba];
    end
	    Mu = K*Mu;
    
    My = K*Aa^(Nx-1)*L;


%% Discrete-time simulation of the PMVC system
tfinal = 30; % total time simulation
N = round( tfinal/Ts ); % total number of samples

    % Reference sequence
    r1(1:Nx+1) = 0; r1(Nx+2:N) = 1; % for y1
    

for k = 1:Nx % initial conditions
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


    %% PMVC
    xa(:,:,k) = (Aa-L*Ca)*xa(:,:,k-1) +Ba*du(k-Nx) +L*y(k-1);
    
        % DU vector from Eq. (18)
        DU = [];
        for i = 1:Nx-1
            DU = [DU; du(k-i) ];
        end

    du(k) = inv(K*Ba+LAM)*( K*[r1(k);0;0] -Mx*xa(:,:,k) ...
        -Mu*DU -My*y(k) );

    %% Testing the LQG instead of PMVC
    %xa(:,:,k) = (Aa-L*Ca)*xa(:,:,k-1) +Ba*du(k-1) +L*y(k-1);
    %du(k) = K*([r1(k);0;0] -xa(:,:,k));

    % Control increment
    u(k) = u(k-1) +du(k);

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

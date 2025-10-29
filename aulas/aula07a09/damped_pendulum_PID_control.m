%% Damped Pendulum Control
clear all; close all; clc;

%% Parameter for the model based on the paper of Lemes et al. (2010)
J = 0.4; % [kg*m^2]
c = 0.2; % damping factor
m = 0.4; % [kg]
g = 9.8; % [m/s^2]
d = 0.05; % [m]
r = 0.4; % [m]

    % Linear Transfer Function approximation
    %                           r
    %  theta(s)/F(s) = -------------------
    %                   J*s^2 +c*s +m*g*d

    % F(s) is the propulsion system force [N]
    % theta(s) is the pendulum angular position [rad]
    Gs = tf(r, [J  c  m*g*d]);
    
    Ts = 0.1; % Sampling time (seconds)
    Gz = c2d(Gs,Ts,'zoh');
       Bz = Gz.num{1};
            b0 = Bz(2); b1 = Bz(3);
       Az = Gz.den{1};
            a1 = Az(2); a2 = Az(3);

%% Digital PID Control
   kp = 1;  ki = .4; kd = 0.6; % PID gains
   
    % Digital PID based on the Backward diff. synthesis
    s0 = kp +ki*Ts +kd/Ts
    s1 = -kp -2*kd/Ts
    s2 = kd/Ts

    % Control system relative stability analysis
    Cz = tf([s0 s1 s2],[1 -1 0],Ts);

    Gdlz = Cz*Gz; % Direct loop system
    w = logspace(-2,5,100);
    figure; margin(Gdlz,w); % Gain and Phase Margins


% Non linear pendulum model
  % x1 is the angular position (rad); x2 is the angular velocity (rad/s).
  % u is the propulsion system force (N)
  % x1(k) = x1(k-1) +Ts*x2(k-1);
  % x2(k) = (1- c*Ts/J)*x2(k-1) -(m*g*d*Ts/J)*sin(x1(k-1)) +(r*Ts/J)*u(k-1);


% System non linear simulation
tfinal = 20; % total simulation time (seconds)
N = round( tfinal/Ts ); % total number of samples

% Reference sequence
  yr(1:10) = 0; yr(11:N) = 5*(pi/180); % rad

% Load disturbance sequence
  v(1:N/2) = 0; v(1+(N/2):N) = 0.5*(pi/180); % rad

% Gaussian Noise disturbance sequence
  v2 = 1*wgn(1,N,1e-4,'linear'); % Power in Watts

% Initial conditions
    x1(1:2)=0;
    x2(1:2)=0;
    u(1:2)=0; e(1:2)=0;

for k = 3:N
% Non linear pendulum model
  x1(k) = x1(k-1) +Ts*x2(k-1);
  x2(k) = (1- c*Ts/J)*x2(k-1) -(m*g*d*Ts/J)*sin(x1(k-1)) +(r*Ts/J)*u(k-1);
  y(k) = x1(k) +v(k) +v2(k);

  % Controller
  e(k) = yr(k) -y(k);
  u(k) = u(k-1) +s0*e(k) +s1*e(k-1) +s2*e(k-2);

  if u(k) >= 0.25
      u(k) = 0.25;
  elseif u(k) <= 0
      u(k) = 0;
  end
end


% System linear simulation
% Initial conditions
    yL(1)=0; yL(2)=0;
    uL(1:2)=0; % Remark: uL refers to the linear control signal
    eL(1:2)=0;

for k = 3:N
% Linear pendulum model
  yL(k) = -a1*yL(k-1) -a2*yL(k-2) +b0*uL(k-1) +b1*uL(k-2) ...
         +v(k) +a1*v(k-1) +a2*v(k-2) ...
         +v2(k) +a1*v2(k-1) +a2*v2(k-2);

  % Controller
  eL(k) = yr(k) -yL(k);
  uL(k) = uL(k-1) +s0*eL(k) +s1*eL(k-1) +s2*eL(k-2);
end



% Plots
t = 0:Ts:N*Ts-Ts;
figure;
subplot(311)
    plot(t,yr,'k',t,y,'b',t,yL,'r');
    ylabel('x1(t) and Linear y(t) (rad)');
    legend('Ref.','Non linear','Linear');
subplot(312)
    plot(t,(180/pi)*yr,'k',t,(180/pi)*y,'b',t,(180/pi)*yL,'r');
    ylabel('x1(t) and Linear y(t) (deg)');
    legend('Ref.','Non linear','Linear');
subplot(313)
    plot(t,u,'b', t,uL,'--r');
    ylabel('u(t) (N)');
    legend('Non linear','Linear');






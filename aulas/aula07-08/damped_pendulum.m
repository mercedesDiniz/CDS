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
    
    Ts = 0.001; % Sampling time (seconds)
    Gz = c2d(Gs,Ts,'zoh');
       Bz = Gz.num{1};
            b0 = Bz(2); b1 = Bz(3);
       Az = Gz.den{1};
            a1 = Az(2); a2 = Az(3);

% Non linear pendulum model
  % x1 is the angular position (rad); x2 is the angular velocity (rad/s).
  % u is the propulsion system force (N)
  % x1(k) = x1(k-1) +Ts*x2(k-1);
  % x2(k) = (1- c*Ts/J)*x2(k-1) -(m*g*d*Ts/J)*sin(x1(k-1)) +(r*Ts/J)*u(k-1);


% System non linear simulation
tfinal = 20; % total simulation time (seconds)
N = round( tfinal/Ts ); % total number of samples

% Initial conditions
    x1(1)=0;
    x2(1)=0;
    u(1)=0;

for k = 2:N
% Non linear pendulum model
  x1(k) = x1(k-1) +Ts*x2(k-1);
  x2(k) = (1- c*Ts/J)*x2(k-1) -(m*g*d*Ts/J)*sin(x1(k-1)) +(r*Ts/J)*u(k-1);

  % Controller
  u(k) = 0.25; % Control Law in open-loop.
end


% System linear simulation
% Initial conditions
    y(1)=0; y(2)=0;
    uL(1:2)=0; % Remark: uL refers to the linear control signal

for k = 3:N
% Linear pendulum model
  y(k) = -a1*y(k-1) -a2*y(k-2) +b0*uL(k-1) +b1*uL(k-2);

  % Controller
  uL(k) = 0.25; % Control Law in open-loop.
end



% Plots
t = 0:Ts:N*Ts-Ts;
subplot(311)
    plot(t,x1,'b',t,y,'r');
    ylabel('x1(t) and Linear y(t) (rad)');
    legend('Non linear','Linear');
subplot(312)
    plot(t,(180/pi)*x1,'b',t,(180/pi)*y,'r');
    ylabel('x1(t) and Linear y(t) (deg)');
    legend('Non linear','Linear');
subplot(313)
    plot(t,u,'b', t,uL,'--r');
    ylabel('u(t) (N)');
    legend('Non linear','Linear');






%% Simulation of a second-order discrete-time dynamical system
clear all; close all; clc;

%% Second-order continuous-frequency model
ks = 0.9; % static gain [V/V]
wn = 2; % natural frequency [rad/s]
zeta = 0.7; % damping factor
Gs = tf(  [ks*wn^2], [1   2*zeta*wn   wn^2])

%% Zero-Order-Hold (ZOH) discrete-frequency equivalent
ws = 10*wn; % sampling frequency [rad/s]
fs = ws/(2*pi); % sampling frequency [ Hz ]
Ts = 1/fs % Sampling period [ seconds ]

    Gz = c2d( Gs, Ts )
    Bz = Gz.num{1};
        b0 = Bz(2); b1 = Bz(3);
    Az = Gz.den{1};
        a1 = Az(2); a2 = Az(3);
        
    roots_Az = roots(Az);

%% Testing the solution of the homogeneous difference
% equation of the system
    % Solving the values for c_i, i=[1;2].
      % for y(0)=1 and y(1)=0
      
      % y(0)= 1 = c_1*( roots_Az(1) )^0 +c_2*( roots_Az(2) )^0
      %       1 = c_1 +c_2;
      
      % y(1)= 0 = c_1*( roots_Az(1) )^1 +c_2*( roots_Az(2) )^1
      %       0 = c_1*( 0.5804 + 0.2794i ) +c_2*( 0.5804 - 0.2794i )
      %       0 = 0.5804*(c_1 +c_2) +0.2794i*c_1 -0.2794i*c_2

      syms c_1 c_2;
      [Sc_1,Sc_2] = solve( c_1 +c_2 == 1, 0.2794*i*c_1 -0.2794*i*c_2 == -0.5804)

      k=10; yk = Sc_1*( roots_Az(1) )^k +Sc_2*( roots_Az(2) )^k;
    
%% Numerical simulation by solving the difference equation
tfinal = 5; % total simulation time in seconds
N = round( tfinal/Ts ); % total number of iterations
time_k = 0:1:N-1;
for k = 1:N
   y(k) = Sc_1*( roots_Az(1) )^time_k(k) ...
          +Sc_2*( roots_Az(2) )^time_k(k);
end
scatter(time_k,y); grid;


%% Numerical simulation of the discrete-time dynamical system
na = 2;
% initial conditions
    y2(1)=1; y2(2)=0;
% follow-up iterations
for k = 1:N
    y2(k+2) = -a1*y2(k+1) -a2*y2(k);
end
hold; scatter(time_k,y2(1:N),7);
ylabel('y(k)'); xlabel('k');
legend('By difference equation solution', ...
       'By difference equation dimulation');
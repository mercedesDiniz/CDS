%% RST Control:
% Example of a 1st-order plant to be controlled
% by a RST structure to impose a desired
% 1st-order system dynamics.
clear all; close all; clc;

%% Plant
tau = 10; % time constant in seconds
Gs = tf(1,[tau 1]);

%% Discrete ZOH equivalent
Ts = 1; % sampling time in seconds
Gz = c2d(Gs,Ts);
    Az = Gz.den{1}; a1 = Az(2);
    Bz = Gz.num{1}; b0 = Bz(2);

%% RST Control method based on Astrom and Wittenmark (2011)
Ho = Az;
    % Defining Hc based on a desired closed-loop system
    tau_cl = 3; % closed-loop time constant
    Gdes = tf(1,[tau_cl  1]);
    Gdesz = c2d(Gdes,Ts);
    Hc = Gdesz.den{1};

    % The pole-assignment polynomial is
    H = conv(Hc,Ho);
        h1 = H(2); h2 = H(3);

    % R(z) and S(z) polynomials
    s0 = (h1 -a1+1)/b0;
    s1 = (h2 +a1)/b0;

    r0 = 1;
    r1 = -1;
    % Remark: the settings for the R(z) polynomial
    % came from the previous knowledge on PI digital
    % synthesis based on the Backward approximation method.

    % T(z) polynomial
    toff = sum(Hc)/sum(Bz); % DC pre-compensator
    Tz = toff*Ho; t0 = Tz(1); t1 = Tz(2);

%% Discrete time domain simulation
tfinal = 50; % total time [s]
N = round(tfinal/Ts); % number of samples
t = 0:Ts:N*Ts-Ts; % time vector for the plots

    % Reference sequence
    yr(1:2)=0; yr(3:N) = 1;

% Initial conditions
y(1) = 0; u(1) = 0;

for k = 2:N
    y(k) = -a1*y(k-1) +b0*u(k-1);

    % RST controller
    u(k) = u(k-1) +t0*yr(k) +t1*yr(k-1) ...
        -s0*y(k) -s1*y(k-1);
end

subplot(211)
    plot(t,yr,':k',t,y,'b');
subplot(212)
    plot(t,u,'b');
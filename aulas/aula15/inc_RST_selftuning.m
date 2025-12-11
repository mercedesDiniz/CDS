%% Self-tuning control using an Incremental RST control
% structure.
clear all; close all; clc;

%% Loading a previously identified model to start the
% adaptive controller.
load('modelo_Gz.mat');
Gz = modelo.Gz;
Ts = Gz.Ts; % sampling time
    Az = Gz.den{1}
        a1 = Az(2); a2 = Az(3);
    Bz = Gz.num{1}
        b0 = Bz(2); b1 = Bz(3); b0b = b0+b1;

    % Augmenting the model to include the discrete difference
    % operator Delta = 1-z^-1
    DAz = conv(Az,[1 -1])
        da1 = DAz(2); da2 = DAz(3); da3 = DAz(4);


%% RST Controller
Hoz = Az; % Observer polynomial (or dummy poles polynomial)
    % Desired closed-loop poles
    wcl = 1; % selected closed-loop frequency [rad/s]
    zetacl = 1; % desired damping factor
    Gcl = tf(wcl^2,[1   2*zetacl*wcl  wcl^2]);
    Gzcl = c2d(Gcl,Ts);
    Hcz = Gzcl.den{1}; % desired closed-loop polynomial

Hz = conv(Hcz,Hoz);
    h1 = Hz(2); h2 = Hz(3); h3 = Hz(4); h4 = Hz(5);

    % R(z) and S(z) polynomials
    r1 = h4/da3;
    s0 = ( h1-(r1+da1) )/b0b;
    s1 = ( h2-(da1*r1+da2) )/b0b;
    s2 = ( h3-(da2*r1+da3) )/b0b;

    % T(z)
    toff = sum(Hcz)/sum(Bz);
    Tz = toff*Hoz;
        t0 = Tz(1); t1 = Tz(2); t2 = Tz(3);

%% Discrete-time domain simulation
tfinal = 20;
N = round( tfinal/Ts );
t = 0:Ts:N*Ts-Ts;

    % Reference sequence
    yr(1:5)=0;
    yr(6: round( N/3 ))=1;
    yr(1+round( N/3 ): round(2*N/3) )=2.5;
    yr(1+round( 2*N/3 ): N )= 4;

for k = 1:4
    % Simulated variables
    ym(k) = 0;
    um(k) = 0;

    em(k) = 0;
    dum(k) = 0;

    % Experimental variables
    y(k) = 0;
    u(k) = 0;

    e(k) = 0;
    du(k) = 0;
    yest(k) = 0;

    %% Using different variables for the self-tuning
    % controller in order to compare its results to the
    % ones with a fixed set of parameters.
    r1a(k) = r1;
    s0a(k) = s0; s1a(k) = s1; s2a(k) = s2;
    t0a(k) = t0; t1a(k) = t1; t2a(k) = t2;
end
% RLS initial params.
    P = 100*eye(3,3);
% P =[ 0.2971   -0.2609    0.0011    0.0348;
%    -0.2609    0.2396   -0.0012   -0.0207;
%     0.0011   -0.0012    0.0478   -0.0475;
%     0.0348   -0.0207   -0.0475    0.0611];
    theta = [a1 a2 b0]';

daqduino_start('COM4'); % Connect DAQ
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Prepare a Graphic windows for animated real-time plots
    figure;subplot(211);
        plot(NaN,NaN); % empty plot
          animated_y = animatedline;
        ylabel('Output (V)'); xlabel('Time (s)'); hold;
       subplot(212);
        plot(NaN,NaN); % empty plot
          animated_u = animatedline;
        ylabel('Input (V)'); xlabel('Time (s)'); hold;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 5:N
    % Simulation 
    ym(k) = -a1*ym(k-1) -a2*ym(k-2) +b0*um(k-1) +b1*um(k-2);



    % % RST Problematic!
    % dum(k) = -r1*dum(k-1) +t0*yr(k) +t1*yr(k-1) +t2*yr(k-2) ...
    %     -s0*ym(k) -s1*ym(k-1) -s2*ym(k-2);
    % um(k) = um(k-1) +dum(k);

    % PID-IMC for 1st-order in closed-loop
    s0 = (1-exp(-Ts/.5))/b0;
    s1 = s0*a1;
    s2 = s0*a2;
    em(k) = yr(k)-ym(k);
    dum(k) = s0*em(k) +s1*em(k-1) +s2*em(k-2);
    um(k) = um(k-1) +dum(k);

    % Experimental self-tuning control
    y(k) = daqduino_read();
    %y(k)=-a1*y(k-1) -a2*y(k-2) +b0*u(k-1) +b1*u(k-2);

    % Recursive Least-Squares estimator
    phi = [ -y(k-1) -y(k-2) u(k-1) ]';
    L = P*phi*inv(1+phi'*P*phi);
    yest(k) = phi'*theta; % RLS estimated output
    theta = theta +L*( y(k) -yest(k) );
    P = P -L*phi'*P;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Parametric adaptation
        a1a = theta(1);
        a2a = theta(2);
        b0a = theta(3);
        % b1a = theta(4);

        DAza = conv([1 a1a a2a],[1 -1]);
            da1a = DAza(2); da2a = DAza(3); da3a = DAza(4);
        b0ba = b0a;%+b1a;

    % R(z) and S(z) polynomials
    r1a(k) = h4/da3a;
    s0a(k) = ( h1-(r1a(k)+da1a) )/b0ba;
    s1a(k) = ( h2-(da1a*r1a(k)+da2a) )/b0ba;
    s2a(k) = ( h3-(da2a*r1a(k)+da3a) )/b0ba;

    % T(z)
    toffa = sum(Hcz)/b0ba;
    Tza = toffa*Hoz;
        t0a(k) = Tza(1); t1a(k) = Tza(2); t2a(k) = Tza(3);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
        u(k) = yr(k);
        du(k) = u(k) -u(k-1);

 % if k > round(1/Ts)

    % Problematic! RST
    du(k) = -r1a(k)*du(k-1) ...
        +t0a(k)*yr(k) +t1a(k)*yr(k-1) +t2a(k)*yr(k-2) ...
        -s0a(k)*y(k) -s1a(k)*y(k-1) -s2a(k)*y(k-2);


    % PID-IMC for 1st-order in closed-loop
    s0a(k) = (1-exp(-Ts/.5))/b0ba;
    s1a(k) = s0a(k)*a1a;
    s2a(k) = s0a(k)*a2a;
    e(k) = yr(k)-y(k);
    du(k) = s0a(k)*e(k) +s1a(k)*e(k-1) +s2a(k)*e(k-2);
    

    % % Naive saturation
    % if du(k) < -5
    %     du(k) = -5;
    % elseif du(k) > 5
    %     du(k) = 5;
    % end

    u(k) = u(k-1) +du(k);

    % Naive saturation
    if u(k) < 0
        u(k) = 0;
    elseif u(k) > 5
        u(k) = 5;
    end
 % end
    daqduino_write( u(k), Ts );
 

    % Updates the real-time plots
    subplot(211);
        addpoints(animated_y, t(k), y(k));
        drawnow limitrate;
    subplot(212);
        addpoints(animated_u, t(k), u(k));
        drawnow limitrate;
end
daqduino_end; % Disconnect DAQ

% Plots
figure;
subplot(211)
    plot(t,yr,'k'); hold;
    plot(t,ym,'r');
    plot(t,y,'b');
    legend('Ref.','Model','Real');
subplot(212)
    plot(t,um,'r'); hold;
    plot(t,u,'b');
    legend('Model','Real');
   




%% System identification using the LQ (Least-Squares)
clear all; close all; clc;

%% Loading datalog
datalog = load('datalog_tuy.txt');
t = datalog(:,1);
u = datalog(:,2);
y = datalog(:,3);
Ts = t(2)-t(1); % Sampling time [s]
N = length(y); % total number of samples

%plot(t,y); hold; plot(t,u,'r');

%% Range selection for identification purposes
t1 = Ts; % start time [s]
t2 = 2.5; % end time [s]
N1 = round(t1/Ts); % start sample
N2 = round(t2/Ts); % end sample

ui = u(N1:N2);
yi = y(N1:N2);
Ni = length(yi);
plot(yi); hold; plot(ui,'r');
title('Selected range for the ident. process');

%% Least-squares estimator

PHI = [];
for k = 3:Ni
    PHI = [PHI; [-y(k-1) -y(k-2) u(k-1) u(k-2)] ];
end
disp('Covariance Matrix:'); inv(PHI'*PHI)

disp('Estimated parameters: [a1 a2 b0 b1]^T=');
theta = inv(PHI'*PHI)*PHI'*yi(3:Ni)

%% Model check and validation
a1 = theta(1); a2 = theta(2);
b0 = theta(3); b1 = theta(4);
Gz = tf([0 b0 b1],[1 a1 a2],Ts)
figure; step(Gz); title('Model check');

    % Validation (model vs real data)
    ym(1:2) = y(1:2); % Initial conditions
    for k = 3:N
        ym(k) = -a1*ym(k-1) -a2*ym(k-2) +b0*u(k-1) +b1*u(k-2);
    end
    figure;
    plot(t,y); hold; plot(t,ym,'r');
    legend('Real data','Model data');
    ylabel('Output (V)');

%% Recursive Least-Squares
clear theta a1 a2 b0 b1;

theta = [ 0.01; 0.01; 0.01; 0.01 ];
P = 1e3*eye(4,4);

for k = 3:N
    
    % RLS
    % if abs(u(k)-u(k-1)) > 0.5 % Covar Matrix update
    %     P = 1e3*eye(4,4);
    % end
    phi = [ -y(k-1) -y(k-2) u(k-1) u(k-2) ]';
    L = P*phi*inv(1+phi'*P*phi);
    yest(k) = phi'*theta; % RLS estimated output
    theta = theta +L*( y(k) -yest(k) );
    P = P -L*phi'*P;

        % Parametric adaptation
        a1(k) = theta(1);
        a2(k) = theta(2);
        b0(k) = theta(3);
        b1(k) = theta(4);

        % Trace of P to asses the power of the est. error
        trP(k) = trace(P);
end
figure; plot(t,y,'b',t,yest,'-r');
legend('Real Data','RLS Estimated'); xlabel('Time (s)');

figure;
    subplot(411);
        plot(t,a1); ylabel('a1(t)');
    subplot(412);
        plot(t,a2); ylabel('a2(t)');
    subplot(413);
        plot(t,b0); ylabel('b0(t)');
    subplot(414);
        plot(t,b1); ylabel('b1(t)'); xlabel('Time (s)');

figure; plot(t,trP); title('Trace of P along the time');
xlabel('Time (s)');

%% Model check and validation
clear a1 a2 b0 b1 ym;
a1 = theta(1); a2 = theta(2);
b0 = theta(3); b1 = theta(4);
Gz = tf([0 b0 b1],[1 a1 a2],Ts)
figure; step(Gz); title('Model check after the RLS');

    % Validation (model vs real data)
    ym(1:2) = y(1:2); % Initial conditions
    for k = 3:N
        ym(k) = -a1*ym(k-1) -a2*ym(k-2) +b0*u(k-1) +b1*u(k-2);
    end
    figure;
    plot(t,y); hold; plot(t,ym,'r');
    legend('Real data','Model data after RLS');
    ylabel('Output (V)');

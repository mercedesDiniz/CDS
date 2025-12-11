%% Data Acquisition for the circuit system
clear all; close all; clc;


%% Experimental setup
ttotal = 8; % total time in seconds
Ts = 0.03; % sampling time in seconds
t = 0:Ts:ttotal; % time vector for the plots
N = length(t); % total number of samples

    % Input sequence to excite the servo-system
    u=2+square(2*pi*0.25*t);
    u(1:15)=0;


%% Experimental Data-Acquisition Loop
daqduino_start('COM4'); % Connect to the DAQ-Device

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

for k = 1:N % DAQ-Loop
tic % tic-toc to check loop-time
    y(k) = daqduino_read(); %A/D

    daqduino_write(u(k), Ts);
    
    % Updates the real-time plots
    subplot(211);
        addpoints(animated_y, t(k), y(k));
        drawnow limitrate;
    subplot(212);
        addpoints(animated_u, t(k), u(k));
        drawnow limitrate;

elapsed_time(k) = toc; % tic-toc to check loop-time
end
daqduino_end; % Disconnect from DAQ

figure;
subplot(211)
    plot(t,y); ylabel('Output (V)'); xlabel('Time (s)');
subplot(212)
    plot(t,u); ylabel('Input (V)'); xlabel('Time (s)');


%% Assessing the sampling mean time
disp('Mean of the sampling time:');
disp( mean(elapsed_time) );

%% Saving the datalog
datalog = [t' u' y'];
save 'datalog_tuy.txt' datalog -ascii
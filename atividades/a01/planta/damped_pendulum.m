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


% Non linear pendulum model
% 
clear, clc, close all;

load CL_CD.mat
init.tol = 1e-2;
init.g = -3.69; % m/s^2
init.m = 1.8; %kg
init.F_grav_i = [0; 0; init.m*init.g]; % N
init.d_com_lo = [0;0;0.05]; %m
init.d_com_up = [0;0;0.10]; %m
init.r = 0.6; %m
init.c = 0.24; %m
init.I = [0.21 0 0;0 0.288 0;0 0 0.278]; %kgms
init.K_lift = 1;
init.K_drag = 0.9999376;
init.alpha_max = deg2rad(10);
init.omega_max = 2800*2*pi/60;
init.K_P = 300;
init.K_D = 50;

init.abstr_level = input("Choose the abstraction level:\n" + ...
    "0-->Total forces and torques as inputs\n" + ...
    "1-->Blades speed and trust direction as inputs and constant lift and drag coefficients\n" + ...
    "2-->Blades speed and trust direction as inputs and rotor modelling\n> ");
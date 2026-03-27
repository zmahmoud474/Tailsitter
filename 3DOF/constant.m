clc;clearvars;close all
% run this before runing the simulink model to load the constants

m=1.5;%kg
I=0.4;
g=9.81;%m/s^2
Rho = 1.225;

%downwash constants

Dinch=9.6;%inches
D = Dinch*2.54/100;%in meters
Adisk = pi*(D/2)^2; %m^2



%Aero constants

Clw_alpha = 2*pi;
semi_span = 0.8;

y0_Ele = 0.65; 
yi_Ele = 0.2;

Ele_length = y0_Ele - yi_Ele;
cw = 0.2;%m
cEle = 0.05;
sigma_Ele = acos(2*cEle/cw-1);
epsilon = 1-((sigma_Ele-sin(sigma_Ele)) / pi);
Cm_delta=(sin(2*sigma_Ele)-2*sin(sigma_Ele))/4; %thin aerofoil theory

rho = 1.225;

As_P = (D/2)*cw; %here we make a rectanglar area were the of the area is half the propeller diameter
b = 1.5;%Span in meter
Aw = b*cw;
As_free = Aw - 2*As_P;
Tau_Ele = 0.45;

Xac = 0.25;
Xcg = 0;

tau_act = 1/4;%actuator "delay" 


%% initial conditions & hover constants
%hover
q_0 = 0;%rad/sec
x_0 = 0;%m
z_0 = 0;%m
Vx_0 = 0;%m/s
Vz_0 = 0;%m/s
theta_0 = pi/2;%rad
%actuators
T0 = m*g; %thrust required at theta=pi/2 to keep the UAV from accelrating 
delta_0 = 0;





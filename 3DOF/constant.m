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
e=0.9;
AR=b^2/(Aw);
K=1/(pi*e*AR);
CD_0=0.0025;
CL_0=0.04;
Inc=0; %rad

Xac = 0.25;
Xcg = 0.2;

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


%% Horizontal Flight Trim Calculation
%  Define target flight condition
V_cruise = 18.5; % Desired cruise speed in m/s (Adjust as needed)
q_bar = 0.5 * Rho * V_cruise^2; % Dynamic pressure


M_lift = m * g * (Xcg - Xac); 
delta_trim = -M_lift / (q_bar * Aw * cw * Cm_delta);

% 3. Calculate Trim Angle of Attack (alpha_trim)
% L = q_bar * Aw * Cl_trim = m * g
CL_req = (m * g) / (q_bar * Aw);

% CL_trim = Clw_alpha * (alpha_trim + Tau_Ele * delta_trim)
alpha_trim = (CL_req / Clw_alpha) - (Tau_Ele * delta_trim);

% 4. Calculate Trim Thrust (Thrust_trim)
% Thrust must equal aerodynamic Drag. 
% Note: Adding basic Drag assumptions since they are missing from the init vars.
Cd0 = 0.02; % Estimated parasitic drag coefficient (Update with your actual value)
e = 0.8;    % Oswald efficiency factor
AR = b^2 / Aw; % Aspect ratio
K = 1 / (pi * e * AR); % Induced drag factor

Cd_trim = Cd0 + K * CL_req^2; % Total drag coefficient
Drag = q_bar * Aw * Cd_trim;
Thrust_trim = Drag;

% 5. Display the calculated trim values in the Command Window
fprintf('\n--- Horizontal Trim Conditions at V = %.1f m/s ---\n', V_cruise);
fprintf('Trim Angle of Attack (alpha) : %.4f rad  (%.2f deg)\n', alpha_trim, rad2deg(alpha_trim));
fprintf('Trim Elevator (delta)        : %.4f rad  (%.2f deg)\n', delta_trim, rad2deg(delta_trim));
fprintf('Trim Thrust Required         : %.2f N\n', Thrust_trim);
fprintf('--------------------------------------------------\n');


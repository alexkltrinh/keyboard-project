%%ME322 Project - Alex Trinh 
clc; 
clear; 
close all; 

shaft_prop = struct();

%Material Properties 
Sy = 525;  % MPa 
Sut =  800; % MPA  

%Gear Masses 
shaft_prop.O.mass = 12500*10^-3;
shaft_prop.C.mass = 2200*10^-3; 
shaft_prop.D.mass = 10500 *10^-3; 


%Angular Speed 
w_TH = 600 + 15*(20+8);
w_TH_rad = (w_TH*2*pi/60);

% Distances along the shaft from Gear O
x = [0:.1:625];
shaft_prop.distX.O = 0.000;    % Gear O 
shaft_prop.distX.A = 0.150;    % Bearing A
shaft_prop.distX.C = 0.350;    % Gear C (150mm + 200mm)
shaft_prop.distX.B = 0.485;    % Bearing B (350mm + 135mm)
shaft_prop.distX.D = 0.625;    % Sprocket D (485mm + 140mm)

%Critical Point Diameters 
D = 38e-3 ; 
shaft_prop.O.diameter = 0.275; 
shaft_prop.A.diameter =  7*D/6; 
shaft_prop.C.diameter = 0.3; 
shaft_prop.B.diameter = 0.485; 
shaft_prop.D.diameter = 0.250; 

%Ordered member diameters from Gear O 
% diameters = [ 0.275, D, 7*D/6, 1.25*D, 0.300, 1.5*D, 0.485, 1.25*D,  0.250]; 
shaft_prop.diameter = [ 0.275, D, 7*D/6, 1.25*D, 0.300, 1.5*D, 0.485, 1.25*D,  0.250]; 

%% Force Balance Equations: 

% Weight Calcs 
g = 9.81; 
Wo = g*shaft_prop.O.mass;
Wc = g*shaft_prop.C.mass; 
Wd = g*shaft_prop.D.mass; 

%Finding input torque 
hp_to_w = 745.7; 

T_input = (4.5*hp_to_w) / w_TH_rad; % Torque at O 
T_gearC = 0.75*T_input; % Torque at C 
T_gearD = 0.25*T_input; % Torque at D

% Calculate the forces acting on each gear
Ft_O = T_input/(0.5*shaft_prop.O.diameter);
Fa_O = 2.25* Ft_O; 
Fr_O = Fa_O/0.625; 

Ft_C = T_gearC / (0.5*shaft_prop.C.diameter); % Force on gear C (Pt)
Fr_C = Ft_C*tand(30); 

Ft_D = T_gearD / (0.5* shaft_prop.D.diameter); % Force on gear D

%Force Balance Calcs 
 



%% Static Strength Analysis 



%Von Mises 

%% Fatigue/Endurance Calculation

ka = %Surface finish 
kb = % Size Factor 
kc = % Load Factor 
kd = % 
ke = % reliability factor
%%ME322 Project - Alex Trinh 
clc; 
clear; 
close all; 

shaft_prop = struct();

%Material Properties 
Sy = 525;  % MPa 
Sut =  800; % MPa  

%Gear Masses 
shaft_prop.O.mass = 12500*10^-3;
shaft_prop.C.mass = 22000*10^-3; 
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
shaft_prop.O.diameter = 0.3; 
shaft_prop.A.diameter =  7*D/6; 
shaft_prop.C.diameter = 0.275; 
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

Mz_O = Fa_O * (0.5 * shaft_prop.O.diameter);

% Resultant Fx Force Balance 
Bx = Fa_O;

% Resultant My,a Force Balance 
Bz = (1/0.335)*(-0.15*Ft_O  + 0.2* Ft_C +0.435*Ft_D);

% Resultant Fz Force Balance
Az = -(-Ft_O + Bz - Ft_D - Ft_C);

% Resultant Mz,A Force Balance 
By = (1/0.335) * ((Fr_O - Wo)*0.15 - Mz_O + (Fr_C + Wc)*0.20 + Wd*0.435);

Ay = -(By + Fr_O - Wo - Fr_C - Wc - Wd);

%% Shear Moment and Moment Diagrams

% =========================================================================
% ME322 Major Project - Shaft Force Analysis & Diagram Plotting
% =========================================================================

clc; 
clear; 
close all; 

%% 1. Shaft Properties and Given Values
shaft_prop = struct();

% Material Properties 
Sy = 525;    % MPa 
Sut = 800;   % MPa  

% Gear Masses [kg] (Corrected Gear C mass to 22.0 kg)
shaft_prop.O.mass = 12500 * 10^-3; 
shaft_prop.C.mass = 22000 * 10^-3; 
shaft_prop.D.mass = 10500 * 10^-3; 

% Angular Speed (1020 rpm converted to rad/s)
w_TH = 600 + 15*(20+8);
w_TH_rad = (w_TH * 2 * pi) / 60;

% Distances along the shaft from Gear O [m]
% (Corrected Sprocket D to 0.585m based on bearing centerline)
x_O = 0.000;    % Gear O 
x_A = 0.150;    % Bearing A
x_C = 0.350;    % Gear C (150mm + 200mm)
x_B = 0.485;    % Bearing B (350mm + 135mm)
x_D = 0.585;    % Sprocket D (485mm + 100mm)

% Critical Point Diameters [m]
% (Corrected Gear O & C swapped diameters, and fixed Bearing B)
D = 38e-3; 
shaft_prop.O.diameter = 0.300; 
shaft_prop.A.diameter = 7*D/6; 
shaft_prop.C.diameter = 0.275; 
shaft_prop.B.diameter = 1.5*D; 
shaft_prop.D.diameter = 0.250; 

%% 2. Force and Torque Calculations
% Component Weights [N]
g = 9.81; 
Wo = g * shaft_prop.O.mass;
Wc = g * shaft_prop.C.mass; 
Wd = g * shaft_prop.D.mass; 

% Finding Input Torques [N-m]
hp_to_w = 745.7; 
T_input = (4.5 * hp_to_w) / w_TH_rad; % Torque at O 
T_gearC = 0.75 * T_input;             % Torque at C 
T_gearD = 0.25 * T_input;             % Torque at D

% Calculate the forces acting on each gear [N]
Ft_O = T_input / (0.5 * shaft_prop.O.diameter);
Fa_O = 2.25 * Ft_O; 
Fr_O = Fa_O / 0.625; 

% Concentrated bending moment from axial force at Gear O [N-m]
Mz_O = -Fa_O * (0.5 * shaft_prop.O.diameter); 

% Forces on Gear C and Sprocket D [N]
Ft_C = T_gearC / (0.5 * shaft_prop.C.diameter); 
Fr_C = Ft_C * tand(30); 
Ft_D = T_gearD / (0.5 * shaft_prop.D.diameter); 

%% 3. Support Reactions (Static Equilibrium)
% Resultant Fx Force Balance 
Bx = Fa_O;

% Horizontal (XZ) Plane Balances
Bz = (1/0.335) * (-0.15*Ft_O + 0.20*Ft_C + 0.435*Ft_D);
Az = Ft_O + Ft_C + Ft_D - Bz;

% Vertical (XY) Plane Balances
By = (1/0.335) * ((Fr_O - Wo)*0.15 + Mz_O + (Fr_C + Wc)*0.20 + Wd*0.435);
Ay = -(By + (Fr_O - Wo) - (Fr_C + Wc) - Wd);

%% 4. Shear and Bending Moment Equations (Singularity Functions)
% Define shaft domain (0 to 625 mm)
x = linspace(0, 0.625, 2000); 

% XY Plane Forces (Vertical)
Fy_O = Fr_O - Wo;
Fy_A = Ay;
Fy_C = -(Fr_C + Wc);
Fy_B = By;
Fy_D = -Wd;

% XZ Plane Forces (Horizontal)
Fz_O = -Ft_O;
Fz_A = Az;
Fz_C = -Ft_C;
Fz_B = Bz;
Fz_D = -Ft_D;

% Shear Force Arrays [N]
Vy = Fy_O.*(x >= x_O) + Fy_A.*(x >= x_A) + Fy_C.*(x >= x_C) + Fy_B.*(x >= x_B) + Fy_D.*(x >= x_D);
Vz = Fz_O.*(x >= x_O) + Fz_A.*(x >= x_A) + Fz_C.*(x >= x_C) + Fz_B.*(x >= x_B) + Fz_D.*(x >= x_D);

% Bending Moment Arrays [N-m] (Integral of shear)
Mz = Mz_O.*(x >= x_O) + Fy_O.*max(0, x - x_O) + Fy_A.*max(0, x - x_A) + Fy_C.*max(0, x - x_C) + Fy_B.*max(0, x - x_B) + Fy_D.*max(0, x - x_D);
My = Fz_O.*max(0, x - x_O) + Fz_A.*max(0, x - x_A) + Fz_C.*max(0, x - x_C) + Fz_B.*max(0, x - x_B) + Fz_D.*max(0, x - x_D);

% Resultants
V_res = sqrt(Vy.^2 + Vz.^2);
M_res = sqrt(Mz.^2 + My.^2);

%% 5. Plotting the Diagrams
figure('Name', 'Shear and Bending Moment Diagrams', 'Color', 'w', 'Position', [100, 100, 900, 700]);

% Plot 1: Resultant Shear Force
subplot(2,1,1);
plot(x*1000, V_res, 'b-', 'LineWidth', 2);
grid on; hold on;
x_ticks = [x_O, x_A, x_C, x_B, x_D] * 1000;
xline(x_ticks, 'k--', {'Gear O', 'Brg A', 'Gear C', 'Brg B', 'Sprocket D'}, 'LabelOrientation', 'horizontal');
title('Resultant Shear Force Diagram ($V_{res} = \sqrt{V_y^2 + V_z^2}$)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Shear Force, V [N]', 'FontSize', 12);
xlim([0 625]);

% Plot 2: Resultant Bending Moment
subplot(2,1,2);
plot(x*1000, M_res, 'r-', 'LineWidth', 2);
grid on; hold on;
xline(x_ticks, 'k--');
title('Resultant Bending Moment Diagram ($M_{res} = \sqrt{M_y^2 + M_z^2}$)', 'Interpreter', 'latex', 'FontSize', 14);
xlabel('Position along shaft, x [mm]', 'FontSize', 12);
ylabel('Bending Moment, M [N-m]', 'FontSize', 12);
xlim([0 625]);

% Locate and label the maximum moment
[max_M, idx] = max(M_res);
max_x = x(idx) * 1000;
plot(max_x, max_M, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
text(max_x - 40, max_M + 12, sprintf('Max M = %.2f N-m', max_M), 'FontSize', 11, 'FontWeight', 'bold');

disp('Simulation Complete. Bending Moment max value found.');

%% Static Strength Analysis 



%Von Mises 

%% Fatigue/Endurance Calculation

ka = %Surface finish 
kb = % Size Factor 
kc = % Load Factor 
kd = % 
ke = % reliability factor
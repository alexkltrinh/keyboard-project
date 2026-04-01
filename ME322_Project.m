%%ME322 Project - Alex Trinh 
clc; 
clear; 
close all; 

shaft_prop = struct();

%Material Properties 
Sy = 525;  % MPa 
Sut =  800; % MPa  

g = 9.81; 

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
shaft_prop.distX.D = 0.585;    % Sprocket D (485mm + 140mm)

%Critical Point Diameters 
D = 38e-3 ; 
shaft_prop.O.diameter = 0.3; 
shaft_prop.A.diameter =  7*D/6; 
shaft_prop.C.diameter = 0.275; 
shaft_prop.B.diameter = 1.5*D; 
shaft_prop.D.diameter = 0.250; 

%Ordered member diameters from Gear O 
% diameters = [ 0.275, D, 7*D/6, 1.25*D, 0.300, 1.5*D, 0.485, 1.25*D,  0.250]; 
shaft_prop.diameter = [ 0.275, D, 7*D/6, 1.25*D, 0.300, 1.5*D, 0.485, 1.25*D,  0.250]; 

%% Force Balance Equations: 

% Weight Calcs 
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

Mz_O = - Fa_O * (0.5 * shaft_prop.O.diameter);

% Resultant Fx Force Balance 
Bx = Fa_O;

% Resultant My,a Force Balance 
Bz = (1/0.335)*(-0.15*Ft_O  + 0.2* Ft_C +0.435*Ft_D);

% Resultant Fz Force Balance
Az = -(-Ft_O + Bz - Ft_D - Ft_C);

% Resultant Mz,A Force Balance 
By = (1/0.335) * ((Fr_O - Wo)*0.15 + Mz_O + (Fr_C + Wc)*0.20 + Wd*0.435);
Ay = -(By + (Fr_O - Wo) - (Fr_C + Wc) - Wd);

%% Shear Moment and Moment Diagrams

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
Vy = Fy_O.*(x >= shaft_prop.distX.O) + Fy_A.*(x >= shaft_prop.distX.A) + Fy_C.*(x >= shaft_prop.distX.C) + Fy_B.*(x >= shaft_prop.distX.B) + Fy_D.*(x >= shaft_prop.distX.D);
Vz = Fz_O.*(x >= shaft_prop.distX.O) + Fz_A.*(x >= shaft_prop.distX.A) + Fz_C.*(x >= shaft_prop.distX.C) + Fz_B.*(x >= shaft_prop.distX.B) + Fz_D.*(x >= shaft_prop.distX.D);

% Bending Moment Arrays [N-m] (Integral of shear)
Mz = Mz_O.*(x >= shaft_prop.distX.O) + Fy_O.*max(0, x - shaft_prop.distX.O) + Fy_A.*max(0, x - shaft_prop.distX.A) + Fy_C.*max(0, x - shaft_prop.distX.C) + Fy_B.*max(0, x - shaft_prop.distX.B) + Fy_D.*max(0, x - shaft_prop.distX.D);
My = Fz_O.*max(0, x - shaft_prop.distX.O) + Fz_A.*max(0, x - shaft_prop.distX.A) + Fz_C.*max(0, x - shaft_prop.distX.C) + Fz_B.*max(0, x - shaft_prop.distX.B) + Fz_D.*max(0, x - shaft_prop.distX.D);

% Resultants
V_res = sqrt(Vy.^2 + Vz.^2);
M_res = sqrt(Mz.^2 + My.^2);

%% Shear and Moment Diagrams

figure('Name', 'Shear and Bending Moment Diagrams', 'Color', 'w', 'Position', [100, 100, 900, 700]);

% Resultant Shear Force Plot
subplot(2,1,1);
plot(x*1000, V_res, 'b-', 'LineWidth', 2);
grid on; hold on;
x_ticks = [shaft_prop.distX.O, shaft_prop.distX.A, shaft_prop.distX.C, shaft_prop.distX.B, shaft_prop.distX.D] * 1000;
xline(x_ticks, 'k--', {'Gear O', 'Bearing A', 'Gear C', 'Bearing B', 'Sprocket D'}, 'LabelOrientation', 'horizontal');
title('Resultant Shear Force Diagram ($V_{res} = \sqrt{V_y^2 + V_z^2}$)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Shear Force, V [N]');
xlim([0 625]);


%Resultant Moment Plot
subplot(2,1,2);
plot(x*1000, M_res, 'r-', 'LineWidth', 2);
grid on; hold on;
xline(x_ticks, 'k--');
title('Resultant Bending Moment Diagram ($M_{res} = \sqrt{M_y^2 + M_z^2}$)', 'Interpreter', 'latex', 'FontSize', 14);
xlabel('Position along shaft, x [mm]');
ylabel('Bending Moment, M [N-m]');
xlim([0 625]);

% Locate and label the maximum moment
[max_M, idx] = max(M_res);
max_x = x(idx) * 1000;
plot(max_x, max_M, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
text(max_x - 40, max_M + 12, sprintf('Max M = %.2f N-m', max_M), 'FontSize', 11, 'FontWeight', 'bold');


%% Static Strength Analysis 

%Stress Concentration Factors

%Von Mises 

%% Fatigue/Endurance Calculation
Se_0 = 0.5*Sut;

a = 1.38; 
b = -0.067;

ka = a*Sut^b;      %Surface finish 
% kb = % Size Factor 
kc = 1; % Load Factor 
kd = 1; % Temperature Factor 


Za = ((1.645-1.288)/5)*2 + 1.288;
ke = 1 - 0.08*Za;        % Reliability factor

% Se = ka*kb*kc*kd*ke*Se_0;

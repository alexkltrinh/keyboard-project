

%% 1. Define Input Parameters and Calculated Forces
% Operating speed: 1020 rpm (106.81 rad/s)
% Input power: 4.5 hp = 3355.65 W

% Distances along the shaft from Gear O (in meters)
x_O = 0.000;    % Gear O
x_A = 0.150;    % Bearing A
x_C = 0.350;    % Gear C (150mm + 200mm)
x_B = 0.485;    % Bearing B (350mm + 135mm)
x_D = 0.625;    % Sprocket D (485mm + 140mm)

% Forces in the XY Plane (Vertical) - Units in Newtons [N]
% Positive is upward (+y)
Fy_O = 631.47;          % Radial force (up) - Weight of Gear O (down)
Fy_A = -619.38;         % Reaction at Bearing A
Fy_C = -314.75;         % Radial force (down) - Weight of Gear C (down)
Fy_B = 405.67;          % Reaction at Bearing B
Fy_D = -103.01;         % Weight of Sprocket D (down)

% Concentrated Moment from Axial Force at Gear O [N-m]
Mz_O = 70.70;           % Fa acting at pitch radius creates a z-axis moment

% Forces in the XZ Plane (Horizontal) - Units in Newtons [N]
% Positive is out of the page (+z), Negative is into the page (-z)
Fz_O = -209.47;         % Tangential force at Gear O
Fz_A = 346.07;          % Reaction at Bearing A
Fz_C = -171.35;         % Tangential force at Gear C
Fz_B = 97.55;           % Reaction at Bearing B
Fz_D = -62.80;          % Tangential force at Sprocket D

%% 2. Define the Shaft Domain
% Create an array of 2000 points along the length of the shaft for smooth plotting
x = linspace(0, 0.625, 2000); 

%% 3. Calculate Shear and Bending Moments using Singularity Functions
% Singularity functions (e.g., x >= a) act as step switches, turning 'on' 
% when the evaluation point passes the force location.

% --- XY Plane (Vertical Forces cause Vy shear and Mz bending) ---
Vy = Fy_O.*(x >= x_O) + Fy_A.*(x >= x_A) + Fy_C.*(x >= x_C) + ...
     Fy_B.*(x >= x_B) + Fy_D.*(x >= x_D);

% Bending moment is the integral of shear + any concentrated moments
Mz = Mz_O.*(x >= x_O) + ...
     Fy_O.*max(0, x - x_O) + Fy_A.*max(0, x - x_A) + ...
     Fy_C.*max(0, x - x_C) + Fy_B.*max(0, x - x_B) + ...
     Fy_D.*max(0, x - x_D);

% --- XZ Plane (Horizontal Forces cause Vz shear and My bending) ---
Vz = Fz_O.*(x >= x_O) + Fz_A.*(x >= x_A) + Fz_C.*(x >= x_C) + ...
     Fz_B.*(x >= x_B) + Fz_D.*(x >= x_D);

My = Fz_O.*max(0, x - x_O) + Fz_A.*max(0, x - x_A) + ...
     Fz_C.*max(0, x - x_C) + Fz_B.*max(0, x - x_B) + ...
     Fz_D.*max(0, x - x_D);

%% 4. Calculate Resultants
% Resultant vectors combine the orthogonal plane components
V_res = sqrt(Vy.^2 + Vz.^2);
M_res = sqrt(Mz.^2 + My.^2);

%% 5. Plotting the Results
figure('Name', 'Shaft Shear and Bending Moment Diagrams', 'Color', 'w', 'Position', [100, 100, 900, 700]);

% Plot 1: Resultant Shear Force
subplot(2,1,1);
plot(x*1000, V_res, 'b-', 'LineWidth', 2); % Convert x back to mm for clean axes
grid on; hold on;
% Mark critical nodes
x_ticks = [x_O, x_A, x_C, x_B, x_D] * 1000;
xline(x_ticks, 'k--', {'Gear O', 'Brg A', 'Gear C', 'Brg B', 'Sprock D'}, 'LabelOrientation', 'horizontal');
title('Resultant Shear Force Diagram ($V_{res} = \sqrt{V_y^2 + V_z^2}$)', 'Interpreter', 'latex', 'FontSize', 14);
xlabel('Position along shaft, x [mm]', 'FontSize', 12);
ylabel('Resultant Shear Force, V [N]', 'FontSize', 12);
xlim([0 625]);

% Plot 2: Resultant Bending Moment
subplot(2,1,2);
plot(x*1000, Mz, 'r-', 'LineWidth', 2);
grid on; hold on;
xline(x_ticks, 'k--');
title('Resultant Bending Moment Diagram ($M_{res} = \sqrt{M_y^2 + M_z^2}$)', 'Interpreter', 'latex', 'FontSize', 14);
xlabel('Position along shaft, x [mm]', 'FontSize', 12);
ylabel('Resultant Moment, M [N-m]', 'FontSize', 12);
xlim([0 625]);

% Find and display the maximum resultant moment
[max_M, idx] = max(M_res);
max_x = x(idx) * 1000;
plot(max_x, max_M, 'ko', 'MarkerFaceColor', 'k');
text(max_x - 30, max_M + 15, sprintf('Max M = %.2f N-m', max_M), 'FontSize', 11, 'FontWeight', 'bold');

disp('Simulation Complete. Check the generated figure for the resulting FBD mappings.');
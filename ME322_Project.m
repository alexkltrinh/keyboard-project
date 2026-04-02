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
shaft_prop.distX.Dt = 0.585;   % Sprocket D (485mm + 140mm)
shaft_prop.distX.Dw = 0.595;   % Sprocket D Center of Mass plane (Wd)

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
By = (1/0.335) * ((Fr_O - Wo)*0.15 + Mz_O + (Fr_C + Wc)*0.20 + Wd*0.445);
Ay = -(By + (Fr_O - Wo) - (Fr_C + Wc) - Wd);

%% Shear Moment and Moment Diagrams

shear_moment_prop = struct();

% Define shaft domain (0 to 625 mm)
shear_moment_prop.x = linspace(0, 0.625, 2000); 

% XY Plane Forces (Vertical)
shear_moment_prop.Fy_O = Fr_O - Wo;
shear_moment_prop.Fy_A = Ay;
shear_moment_prop.Fy_C = -(Fr_C + Wc);
shear_moment_prop.Fy_B = By;
shear_moment_prop.Fy_Dw = -Wd;

% XZ Plane Forces (Horizontal)
shear_moment_prop.Fz_O = -Ft_O;
shear_moment_prop.Fz_A = Az;
shear_moment_prop.Fz_C = -Ft_C;
shear_moment_prop.Fz_B = Bz;
shear_moment_prop.Fz_Dt = -Ft_D;

% Shear Force Arrays [N]
shear_moment_prop.Vy = shear_moment_prop.Fy_O.*(shear_moment_prop.x >= shaft_prop.distX.O) + shear_moment_prop.Fy_A.*(shear_moment_prop.x >= shaft_prop.distX.A) + shear_moment_prop.Fy_C.*(shear_moment_prop.x >= shaft_prop.distX.C) + shear_moment_prop.Fy_B.*(shear_moment_prop.x >= shaft_prop.distX.B) + shear_moment_prop.Fy_Dw.*(shear_moment_prop.x >= shaft_prop.distX.Dw);
shear_moment_prop.Vz = shear_moment_prop.Fz_O.*(shear_moment_prop.x >= shaft_prop.distX.O) + shear_moment_prop.Fz_A.*(shear_moment_prop.x >= shaft_prop.distX.A) + shear_moment_prop.Fz_C.*(shear_moment_prop.x >= shaft_prop.distX.C) + shear_moment_prop.Fz_B.*(shear_moment_prop.x >= shaft_prop.distX.B) + shear_moment_prop.Fz_Dt.*(shear_moment_prop.x >= shaft_prop.distX.Dt);

% Bending Moment Arrays [N-m] (Integral of shear)
shear_moment_prop.Mz = Mz_O.*(shear_moment_prop.x >= shaft_prop.distX.O) + shear_moment_prop.Fy_O.*max(0, shear_moment_prop.x - shaft_prop.distX.O) + shear_moment_prop.Fy_A.*max(0, shear_moment_prop.x - shaft_prop.distX.A) + shear_moment_prop.Fy_C.*max(0, shear_moment_prop.x - shaft_prop.distX.C) + shear_moment_prop.Fy_B.*max(0, shear_moment_prop.x - shaft_prop.distX.B) + shear_moment_prop.Fy_Dw.*max(0, shear_moment_prop.x - shaft_prop.distX.Dw);
shear_moment_prop.My = shear_moment_prop.Fz_O.*max(0, shear_moment_prop.x - shaft_prop.distX.O) + shear_moment_prop.Fz_A.*max(0, shear_moment_prop.x - shaft_prop.distX.A) + shear_moment_prop.Fz_C.*max(0, shear_moment_prop.x - shaft_prop.distX.C) + shear_moment_prop.Fz_B.*max(0, shear_moment_prop.x - shaft_prop.distX.B) + shear_moment_prop.Fz_Dt.*max(0, shear_moment_prop.x - shaft_prop.distX.Dt);

% Resultants
shear_moment_prop.V_res = sqrt(shear_moment_prop.Vy.^2 + shear_moment_prop.Vz.^2);
shear_moment_prop.M_res = sqrt(shear_moment_prop.Mz.^2 + shear_moment_prop.My.^2);

%% Shear and Moment Diagrams

figure('Name', 'Shear and Bending Moment Diagrams', 'Color', 'w', 'Position', [100, 100, 900, 700]);

% Resultant Shear Force Plot
subplot(2,1,1);
plot(shear_moment_prop.x*1000, shear_moment_prop.V_res, 'b-', 'LineWidth', 2);
grid on; hold on;
x_ticks = [shaft_prop.distX.O, shaft_prop.distX.A, shaft_prop.distX.C, shaft_prop.distX.B, shaft_prop.distX.Dw] * 1000;
xline(x_ticks, 'k--', {'Gear O', 'Bearing A', 'Gear C', 'Bearing B', 'Sprocket D'}, 'LabelOrientation', 'horizontal');
title('Resultant Shear Force Diagram ($V_{res} = \sqrt{V_y^2 + V_z^2}$)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Shear Force, V [N]');
xlim([0 625]);
ylim([0 800]);

%Resultant Moment Plot
subplot(2,1,2);
plot(shear_moment_prop.x*1000, shear_moment_prop.M_res, 'r-', 'LineWidth', 2);
grid on; hold on;
xline(x_ticks, 'k--');
title('Resultant Bending Moment Diagram ($M_{res} = \sqrt{M_y^2 + M_z^2}$)', 'Interpreter', 'latex', 'FontSize', 14);
xlabel('Position along shaft, x [mm]');
ylabel('Bending Moment, M [N-m]');
xlim([0 625]);

% Locate and label the maximum moment
[max_M, idx] = max(shear_moment_prop.M_res);
max_x = shear_moment_prop.x(idx) * 1000;
plot(max_x, max_M, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
text(max_x - 40, max_M + 12, sprintf('Max M = %.2f N-m', max_M), 'FontSize', 11, 'FontWeight', 'bold');


%% Static Strength Analysis - Von Mises at All Stress Concentration Points


% Neuber Correction Factors (Notch Sensitivity, Shigley Eq. 6-35b)
sqrt_a_f  = 1.24 - 2.25e-3*Sut + 1.6e-6*Sut^2 - 4.11e-10*Sut^3;   % Bending & Axial
sqrt_a_fs = 0.958 - 1.83e-3*Sut + 1.43e-6*Sut^2 - 4.11e-10*Sut^3;  % Torsion


% Theoretical SCFs (Shigley Charts A-15)
Kt_key  = 2.14;  Kts_key = 3.00;   % End-milled keyway
Kt_f1   = 2.6;   Kts_f1  = 2.1;    % Sharp shoulder fillet (r/d=0.02, D/d~1.25)
Kt_f2   = 2.1;   Kts_f2  = 1.6;    % Generous shoulder fillet (r/d=0.05, D/d~1.2)

% ========================
% All Critical Stress Concentration Points (from shaft drawing)
% ========================
% Each point: x-position, local shaft diameter (smaller side at fillet),
%             Kt, Kts, and notch/fillet radius ratio r/d
%
% Shaft diameters (left to right):
%   9D/10 -> D -> 7D/6 -> 1.25D -> ... -> 1.5D -> 1.25D -> D
% Keyways at Gear O, Gear C, and Sprocket D
% Shoulder fillets at every diameter step transition

scp_labels = { ...
    '1. Gear O Keyway',            ...
    '2. Shoulder: 9D/10 -> D',     ...
    '3. Shoulder: D -> 7D/6',      ...
    '4. Shoulder: 7D/6 -> 1.25D',  ...
    '5. Gear C Keyway',            ...
    '6. Shoulder: 1.25D -> 1.5D',  ...
    '7. Shoulder: 1.5D -> 1.25D',  ...
    '8. Shoulder: 1.25D -> D',     ...
    '9. Sprocket D Keyway'};

% x-positions [m] (from shaft drawing dimensions)
scp_x   = [0.000,  0.020,  0.170,  0.325,  0.350,  0.375,  0.505,  0.565,  0.585];

% Local shaft diameter at each SCP (smaller diameter at shoulder) [m]
scp_d   = [D,      9*D/10, D,      7*D/6,  1.25*D, 1.25*D, 1.25*D, D,      D    ];

% SCF assignments per point (keyway or fillet type)
scp_Kt  = [Kt_key, Kt_f1,  Kt_f1,  Kt_f2,  Kt_key, Kt_f2,  Kt_f1,  Kt_f1,  Kt_key];
scp_Kts = [Kts_key,Kts_f1, Kts_f1, Kts_f2, Kts_key,Kts_f2, Kts_f1, Kts_f1, Kts_key];
scp_rd  = [0.02,   0.02,   0.02,   0.05,   0.02,   0.05,   0.02,   0.02,   0.02 ];


scp_sigma_vm = zeros(1, length(scp_x));
scp_Ny       = zeros(1, length(scp_x));

fprintf(' Von Mises at All Critical Stress Concentration Points\n');

for i = 1:length(scp_x)
    % --- Local cross-section geometry ---
    d_i = scp_d(i);
    c_i = d_i / 2;
    I_i = (pi * d_i^4) / 64;
    J_i = (pi * d_i^4) / 32;
    A_i = (pi * d_i^2) / 4;

    % --- Internal loads at this x-position ---
    [~, idx_i] = min(abs(shear_moment_prop.x - scp_x(i)));
    M_i = shear_moment_prop.M_res(idx_i);   % Resultant bending moment [N-m]
    V_i = shear_moment_prop.V_res(idx_i);   % Resultant shear force [N]

    % Torque distribution along the shaft:
    %   x <= Gear C  :  T = T_input
    %   Gear C < x <= Sprocket D :  T = T_gearD = 0.25*T_input
    %   x > Sprocket D :  T = 0
    if scp_x(i) <= shaft_prop.distX.C
        T_i = T_input;
    elseif scp_x(i) <= shaft_prop.distX.Dt
        T_i = T_gearD;
    else
        T_i = 0;
    end

    % Axial force (Fa_O acts from Gear O to Bearing B)
    if scp_x(i) <= shaft_prop.distX.B
        Fa_i = Fa_O;
    else
        Fa_i = 0;
    end

    % --- Neuber correction for this notch/fillet radius ---
    r_i   = scp_rd(i) * d_i;   % Notch radius [m]
    Kf_i  = 1 + (scp_Kt(i)  - 1) / (1 + sqrt_a_f  / sqrt(r_i * 1e3));
    Kfs_i = 1 + (scp_Kts(i) - 1) / (1 + sqrt_a_fs / sqrt(r_i * 1e3));

    % --- Nominal stresses [Pa] ---
    sigma_bend  = abs(M_i * c_i / I_i);      % Bending (Mc/I)
    sigma_axial = abs(Fa_i / A_i);            % Axial (uniform across section)
    tau_torsion = abs(T_i  * c_i / J_i);      % Torsion (Tc/J)
    tau_VQIt    = (4 * V_i) / (3 * A_i);      % Transverse shear (4V/3A for solid circle)

    % ==========================================================
    % EVALUATION A: Outer Surface (y = c)
    %   - Max bending, max torsion, zero transverse shear
    %   - SCFs apply (keyway/fillet is at the surface)
    % ==========================================================
    sigma_surf = Kf_i  * (sigma_bend + sigma_axial);
    tau_surf   = Kfs_i * tau_torsion;
    sigma_vm_surf = sqrt(sigma_surf^2 + 3 * tau_surf^2);

    % ==========================================================
    % EVALUATION B: Neutral Axis (y = 0)
    %   - Zero bending, max transverse shear + torsion
    %   - SCFs = 1.0 (notch is at surface, not at center)
    % ==========================================================
    sigma_na = sigma_axial;                       % Only axial (uniform)
    tau_na   = tau_torsion + tau_VQIt;             % Torsion + VQ/It both peak here
    sigma_vm_na = sqrt(sigma_na^2 + 3 * tau_na^2);

    % Governing Von Mises at this SCP
    sigma_vm = max(sigma_vm_surf, sigma_vm_na);

    scp_sigma_vm(i) = sigma_vm;
    scp_Ny(i) = Sy / (sigma_vm / 1e6);

    % Print detailed results for each point
    fprintf('--- %s  (x = %.0f mm, d = %.1f mm) ---\n', ...
        scp_labels{i}, scp_x(i)*1e3, d_i*1e3);
    fprintf('  M = %.2f N-m,  T = %.2f N-m,  Fa = %.1f N,  V = %.1f N\n', ...
        M_i, T_i, Fa_i, V_i);
    fprintf('  Kf = %.3f,  Kfs = %.3f\n', Kf_i, Kfs_i);
    fprintf('  sigma_bend = %.2f MPa,  sigma_axial = %.2f MPa\n', ...
        sigma_bend/1e6, sigma_axial/1e6);
    fprintf('  tau_torsion = %.2f MPa,  tau_VQ/It = %.2f MPa\n', ...
        tau_torsion/1e6, tau_VQIt/1e6);
    fprintf('  VM Surface = %.2f MPa,  VM Neutral Axis = %.2f MPa\n', ...
        sigma_vm_surf/1e6, sigma_vm_na/1e6);
    fprintf('  Governing Von Mises = %.2f MPa\n', sigma_vm / 1e6);
    fprintf('  Static Safety Factor Ny = %.2f\n\n', scp_Ny(i));
end

% ========================
% Governing result across all SCPs
% ========================
[sigma_gov, idx_gov] = max(scp_sigma_vm);
fprintf('============================================================\n');
fprintf(' GOVERNING POINT: %s\n', scp_labels{idx_gov});
fprintf(' Max Von Mises Stress = %.2f MPa\n', sigma_gov / 1e6);
fprintf(' Static Factor of Safety Ny = %.2f\n', Sy / (sigma_gov / 1e6));
fprintf('============================================================\n');

% Preserve variables used by downstream fatigue & power-to-yield sections
sigma_p_critical = sigma_gov;
r_keyway = 0.02 * D;
Kf_key  = 1 + (Kt_key  - 1) / (1 + sqrt_a_f  / sqrt(r_keyway * 1e3));
Kfs_key = 1 + (Kts_key - 1) / (1 + sqrt_a_fs / sqrt(r_keyway * 1e3));

%% Fatigue/Endurance Calculation
fatigue_prop = struct();
Se_0 = 0.5*Sut;
a = 4.51;   % Note: Corrected coefficient for Machined surface from 1.38 to 4.51
b = -0.265; % Note: Corrected exponent for Machined surface from -0.067 to -0.265
Za = ((1.645-1.288)/5)*2 + 1.288;

% Marin Factors
fatigue_prop.mar_factors = [];
fatigue_prop.mar_factors(end+1) = a*Sut^b;                                  % ka surface
fatigue_prop.mar_factors(end+1) = 1.51*(D*10^3)^-0.157;                     % kb size (using D = 38mm)
fatigue_prop.mar_factors(end+1) = 1;                                        % kc load
fatigue_prop.mar_factors(end+1) = 1;                                        % kd temperature
fatigue_prop.mar_factors(end+1) = 1 - 0.08*Za;                              % ke reliability

% Endurance limit
fatigue_prop.Se = Se_0 * prod(fatigue_prop.mar_factors);
fprintf("\nEndurance Limit (Se):  %.2f MPa\n", fatigue_prop.Se);

% Rotating Shaft with Steady Bending, Torsion, and Axial Loads at Gear O
M_m = 0; 
T_a = 0;
M_a = abs(Mz_O);      % Alternating bending moment at Gear O (70.70 N-m)
T_m = T_input;        % Mean steady torque (31.42 N-m)
Fa_m = Fa_O;          % Mean steady axial force (471.31 N)

% Nominal Stresses using shaft diameter D (0.038 m)
sigma_a_nom = (32 * M_a) / (pi * D^3);       % Alternating bending stress
tau_m_nom = (16 * T_m) / (pi * D^3);         % Mean torsional stress
sigma_m_axial = (4 * Fa_m) / (pi * D^2);     % Mean axial stress

% Apply fatigue stress concentration factors (using Kf_key and Kfs_key from your keyway)
% Equivalent Alternating and Mean Von Mises Stresses [Pa]
sigma_a_prime = Kf_key * sigma_a_nom; 
sigma_m_prime = sqrt((Kf_key * sigma_m_axial)^2 + 3 * (Kfs_key * tau_m_nom)^2);

% Convert to MPa to match Sut and Se
sigma_a_prime_MPa = sigma_a_prime / 1e6;
sigma_m_prime_MPa = sigma_m_prime / 1e6;

% Goodman's + DE Safety Factor 
% nf = 1 / ( (sigma_a'/Se) + (sigma_m'/Sut) )
nf = 1 / ((sigma_a_prime_MPa / fatigue_prop.Se) + (sigma_m_prime_MPa / Sut));

fprintf("Equivalent Alternating Stress (sigma_a'): %.2f MPa\n", sigma_a_prime_MPa);
fprintf("Equivalent Mean Stress (sigma_m'):      %.2f MPa\n", sigma_m_prime_MPa);
fprintf("Fatigue Factor of Safety (nf):          %.2f\n", nf);

if nf >= 1
    fprintf("Conclusion: Shaft has INFINITE LIFE.\n");
else
    fprintf("Conclusion: Shaft has FINITE LIFE.\n");
end

%% Minimum Power Level for Static Yield 

% Calculate the Factor of Safety at the critical section (Gear O)
Ny_critical = Sy / (sigma_p_critical / 1e6);

% Current operating power is 4.5 hp
P_current_hp = 4.5; 

% Calculate Power to Yield
P_yield_hp = P_current_hp * Ny_critical;
P_yield_watts = P_yield_hp * 745.7;

fprintf('\n--- Power for Static Yielding ---\n');
fprintf('Power required to cause yielding: %.2f hp (%.2f Watts)\n', P_yield_hp, P_yield_watts);


%% Minimum Diameter for Infinite Life of Shaft

fprintf('\n--- Minimum Diameter Iteration (Target nf = 2.0) ---\n');

% Start with the current diameter in meters
D_iter = 0.038; 
step_size = 0.00001; % Shrink by 0.01 mm each step for 2 decimal place accuracy
nf_iter = nf;        % Start with your current nf (~18.59)

% Variables that do not change with D
Se_prime = 0.5 * Sut;
ka = 4.51 * Sut^-0.265;
Za = 1.288 + 0.4 * (1.645 - 1.288); 
ke = 1 - 0.08 * Za;
kc = 1; kd = 1;

% Iteration Loop
while nf_iter > 2.00
    % Shrink the shaft slightly
    D_iter = D_iter - step_size;
    
    % 1. Update kb (valid for D between 2.79 and 51 mm)
    kb_iter = 1.24 * (D_iter * 1000)^-0.107; 
    Se_iter = ka * kb_iter * kc * kd * ke * Se_prime;
    
    % 2. Update Notch Radius and Neuber Factors
    r_iter = 0.02 * (D_iter * 1000); 
    Kf_iter  = 1 + (Kt_key - 1) / (1 + (sqrt_a_f / sqrt(r_iter)));
    Kfs_iter = 1 + (Kts_key - 1) / (1 + (sqrt_a_fs / sqrt(r_iter)));
    
    % 3. Update Nominal Stresses
    sigma_a_nom = (32 * M_a) / (pi * D_iter^3);       
    tau_m_nom = (16 * T_m) / (pi * D_iter^3);         
    sigma_m_axial = (4 * Fa_m) / (pi * D_iter^2);     
    
    % 4. Update Equivalent Stresses [MPa]
    sigma_a_prime = (Kf_iter * sigma_a_nom) / 1e6; 
    sigma_m_prime = sqrt((Kf_iter * sigma_m_axial)^2 + 3 * (Kfs_iter * tau_m_nom)^2) / 1e6;
    
    % 5. Recalculate Goodman Factor of Safety
    nf_iter = 1 / ((sigma_a_prime / Se_iter) + (sigma_m_prime / Sut));
    
    % Fail-safe to prevent infinite loop if it breaks
    if D_iter < 0.001
        disp('Error: Diameter reached unreasonable size.');
        break;
    end
end

% Convert final diameter to mm and round to 2 decimal places
D_min_mm = round(D_iter * 1000, 2);

fprintf('Target Fatigue Factor of Safety: 2.00\n');
fprintf('Minimum Allowable Diameter (D): %.2f mm\n', D_min_mm);
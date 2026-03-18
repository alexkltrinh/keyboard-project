%%ME322 Project - Alex Trinh 
%Initial Variables 
pd_O = 275;
pd_C = 300;
pd_D = 250;
m_O = 12500*10^-3;
m_C = 2200*10^-3; 
m_D = 10500 *10^-3; 

D = 38; 

w_TH = 600 + 15*(20+8);
w_TH_rad = (w_TH*2*pi/60);

x = [0:.1:625];

%% Force Balance Equations: 

%Finding input torque 
hp_to_w = 745.7; 

T_input = (4.5*hp_to_w) / w_TH_rad; 
T_gearC = 0.75*T_input; 
T_gearD = 0.25*T_input;

% Calculate the forces acting on each gear
F_C = T_gearC / (pd_C / 2); % Force on gear C
F_D = T_gearD / (pd_D / 2); % Force on gear D



%% Static Strength Analysis 



%Von Mises 

%% Fatigue/Endurance Calculation

ka = %Surface finish 
kb = % Size Factor 
kc = % Load Factor 
kd = % 
ke = % reliability factor
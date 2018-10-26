%% SLADe Model

% Inertia:
m           = 1.15;
Ixx         = 0.003833;
Iyy         = 0.007933;
Izz         = 0.0069;

% Geometry:
d           = 0.1275;
r_D         = 0.115*0.375;

% Propulsion:
tau_T = 0.05;
virtual_controls_mat = [ 1,  1,  1,  1; %dT                        
                        -1,  1,  1, -1; %dA
                         1, -1,  1, -1; %dE
                         1,  1, -1, -1]; %dR 
hover_perc = 0.5; % hover percentage of full throttle
max_total_T = 2*m*g; % max thrust (total)

% Aerodynamic:
C_D = [0.025; 0.025; 0.025];
A_D = [1; 1; 1];
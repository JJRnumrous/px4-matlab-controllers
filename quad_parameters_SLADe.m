%% SLADe Model

% Inertia:
m = 15;
Ixx = 0.5;
Iyy = 0.5;
Izz = 0.85;

% Geometry:
d = 0.465;
r_D = 0.18;

% Propulsion:
tau_T = 0.125;
virtual_controls_mat = [1 1 1 1; 0 -1 0 1; 1 0 -1 0; -1 1 -1 1];
hover_perc = 0.5; % hover percentage of full throttle
max_total_T = 2*m*g; % max thrust (total)

% Aerodynamic:
C_D = [0.5; 0.5; 0.5];
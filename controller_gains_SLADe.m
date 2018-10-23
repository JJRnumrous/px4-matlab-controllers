%% SLADe Control Gains

% *************************************************************************
% Setup
% *************************************************************************
PID_FILT_COEFF = 100;

% Trim
T_trim = hover_T/4 * ones(4,1);
vb_trim = zeros(3,1);
wb_trim = zeros(3,1);
ned_trim = zeros(3,1) + [0; 0; 0];
euler_trim = zeros(3,1);

% *************************************************************************
% Longitudinal
% *************************************************************************

% Pitch Rate Damper
q_z1 = 1/10;
q_z2 = 1/6.25;
K_qi = 118.98/max_T;
K_qp = (q_z1+q_z2)*K_qi;
K_qd = (q_z1*q_z2)*K_qi;

q_int_lim = 0.3;

% Pitch Angle Controller
K_theta = 9.1159;

% Longitudinal Velocity Controller
u_z1 = 1/5;
u_z2 = 1/2.73;
K_ui = -1.3583;
K_up = (u_z1+u_z2)*K_ui;
K_ud = (u_z1*u_z2)*K_ui;

% Longitudinal Position Controller
K_x = 1.2844;

% *************************************************************************
% Lateral
% *************************************************************************

% Roll Rate Damper
p_z1 = 1/10;
p_z2 = 1/6.25;
K_pi = 118.98/max_T;
K_pp = (q_z1+q_z2)*K_pi;
K_pd = (q_z1*q_z2)*K_pi;

% Roll Angle Controller
K_phi = 9.1159;

% Lateral Velocity Controller
K_vp = 2;
K_vi = 1;
K_vd = 0.5;

% *************************************************************************
% Directional
% *************************************************************************

% Yaw Rate Damper
r_z1 = 1/10;
r_z2 = 1/6.25;
K_ri = 5828.5/max_T;
K_rp = (q_z1+q_z2)*K_ri;
K_rd = (q_z1*q_z2)*K_ri;

% Yaw Angle Controller
K_psi = 9.7266;

% *************************************************************************
% Heave
% *************************************************************************

% Vertical Climb Rate Controller

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

% Pitch Rate Controller
K_q_P = 1.466;
K_q_I = 0.789;
K_q_D = 0.0135;

% Pitch Angle Controller
K_theta_P = 8;
K_theta_I = 0;
K_theta_D = 0;

% Longitudinal Velocity Controller
K_u_P = 1.6922;
K_u_I = 0.2256;
K_u_D = 0.1128;

% Longitudinal Position Controller
K_n_P = 1.5;
K_n_I = 0;
K_n_D = 0;

% *************************************************************************
% Lateral
% *************************************************************************

% Roll Rate Controller
K_p_P = 1.466;
K_p_I = 0.789;
K_p_D = 0.0135;

% Roll Angle Controller
K_phi_P = 8;
K_phi_I = 0;
K_phi_D = 0;

% Lateral Velocity Controller
K_v_P = 1.6922;
K_v_I = 0.2256;
K_v_D = 0.1128;

% Lateral Position Controller
K_e_P = 1.5;
K_e_I = 0;
K_e_D = 0;

% *************************************************************************
% Directional
% *************************************************************************

% Yaw Rate Damper
K_r_P = 1.354;
K_r_I = 0.564;
K_r_D = 0;

% Yaw Angle Controller
K_psi_P = 4;
K_psi_I = 0;
K_psi_D = 0;

% *************************************************************************
% Heave
% *************************************************************************

% Vertical Velocity Controller
K_w_P = 0.8;
K_w_I = 0.15;
K_w_D = 0;

% Vertical Position Controller
K_d_P = 1.5;
K_d_I = 0;
K_d_D = 0;

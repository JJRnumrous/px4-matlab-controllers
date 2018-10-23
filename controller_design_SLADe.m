%% Trim
virt_T_trim = virtual_controls_mat * T_trim;
virt_T_trim_norm = virt_T_trim ./ (max_T);

%% Linearised State-Space
A_long = [-1/tau_T 0 0 0; d/Iyy 0 0 0; 0 1 0 0; 0 0 -g 0];
B_long = [1/tau_T; 0; 0; 0];

A_lat = [-1/tau_T 0 0 0; d/Ixx 0 0 0; 0 1 0 0; 0 0 g 0];
B_lat = [1/tau_T; 0; 0; 0];

A_dir = [-1/tau_T 0 0; r_D/(R_LD*Izz) 0 0; 0 1 0];
B_dir = [1/tau_T; 0; 0];

A_heave = [-1/tau_T 0 0; -1/m 0 0; 0 1 0];
B_heave = [1/tau_T; 0; 0];
const_heave = [0; g; 0];

%% Control
s = tf('s');

% *************************************************************************
% Longitudinal
% *************************************************************************

% Pitch Rate Damper
C_q = [0 1 0 0];
G_PRD_OL = (C_q*inv(s*eye(4)-A_long)*B_long);
D_PRD = max_T*(K_qp + K_qi/s + K_qd*s);
G_PRD_CL = (D_PRD*G_PRD_OL)/(1+D_PRD*G_PRD_OL);

% Pitch Angle Controller
G_PAC_OL = G_PRD_CL * 1/s;
G_PAC_CL = (K_theta*G_PAC_OL)/(1+K_theta*G_PAC_OL);

% Longitudinal Velocity Controller
G_LVC_OL = G_PAC_CL * -g/s;
D_LVC = K_up + K_ui/s + K_ud*s;
G_LVC_CL = (D_LVC*G_LVC_OL)/(1+D_LVC*G_LVC_OL);

% Longitudinal Position Controller
G_LPC_OL = G_LVC_CL * 1/s;
G_LPC_CL = (G_LPC_OL*K_x)/(1+G_LPC_OL*K_x);

% *************************************************************************
% Lateral
% *************************************************************************

% Roll Rate Damper
C_p = [0 1 0 0];
G_RRD_OL = (C_p*inv(s*eye(4)-A_lat)*B_lat);
D_RRD = max_T * (K_pp + K_pi/s + K_pd*s);
G_RRD_CL = (D_RRD*G_RRD_OL)/(1+D_RRD*G_RRD_OL);

% Roll Angle Controller
C_phi = [0 0 1 0];
G_RAC_OL = G_RRD_CL * 1/s;
G_RAC_CL = (K_phi*G_RAC_OL)/(1+K_phi*G_RAC_OL) * g/s * 0.01 * pi/180;

% Lateral Velocity Controller
C_v = [0 0 0 ];

% *************************************************************************
% Directional
% *************************************************************************

% Yaw Rate Damper
C_r = [0 1 0];
G_YRD_OL = (C_r*inv(s*eye(3)-A_dir)*B_dir);
D_YRD = max_T * (K_rp + K_ri/s + K_rd*s);
G_YRD_CL = (D_YRD*G_YRD_OL)/(1+D_YRD*G_YRD_OL);

% Yaw Angle Controller
C_psi = [0 0 1];
G_YAC_OL = G_YRD_CL * 1/s;
G_YAC_CL = (K_psi*G_YAC_OL)/(1+K_psi*G_YAC_OL);

% *************************************************************************
% Heave
% *************************************************************************

% Vertical Climb Rate Controller

%% Trim
virt_T_trim = virtual_controls_mat * T_trim;
virt_T_trim_norm = virt_T_trim ./ (max_T);


% Longitudinal Dynamic ________________[d_E, q, theta, u]__________________
matLongA     = [-1/tau, 0,  0, 0;
                d/Iyy , 0,  0, 0;
                0     , 1,  0, 0;
                0     , 0, -g, 0];
            
matLongB     = [1/tau ; 0; 0; 0];

% lateral Dynamic _____________________[d_A, p, phi, v]____________________
matLatA      = [-1/tau, 0,  0, 0;
                d/Ixx , 0,  0, 0;
                0     , 1,  0, 0;
                0     , 0,  g, 0];

matLatB      = [1/tau ; 0; 0; 0];

% Directional Dynamic __________________[d_R, r, psi]______________________
matDirA      = [-1/tau         , 0,  0;
                r_D/(R_LD*Izz) , 0,  0;
                0              , 1,  0];

matDirB      = [1/tau ; 0; 0];

% Heave Dynamic ________________________[d_T, w, D]________________________
matHevA      = [-1/tau , 0,  0;
                -1/m   , 0,  0;
                0      , 1,  0];

matHevB      = [1/tau ; 0; 0];

s =tf('s');
%% Longitude


% ___________________ Pitch_rate __________________________________________
C_q = [0,1,0,0];
G_qr = C_q * ((s.*eye(4) - matLongA) \ matLongB);
%rltool(G_qr);
K_q   = K_q_P + K_q_I/s + K_q_D*s;
matRateA = matLongA-matLongB*K_q*C_q;

% ___________________ Pitch _______________________________________________
C_theta = [0,0,1,0];
G_q = C_theta * ((s.*eye(4) - matRateA) \ matLongB);
%rltool(G_q);
K_theta = K_theta_P + K_theta_I/s + K_theta_D*s;

matTheA = matRateA-matLongB*K_theta*C_theta;

% ___________________ Longitudinal Velocity _______________________________
C_u = [0,0,0,1];
G_u = C_u * ((s.*eye(4) - matTheA) \ matLongB);
% rltool(G_u);
K_u = K_u_P + K_u_I/s + K_u_D*s;
% - PID
matVelA = [matTheA+matLongB*K_u*C_u, zeros(4,1);
            0, 0, 0, 1, 0];
matVelB = [matLongB; 0];

% ___________________ Longitudinal Position________________________________
C_n = [0,0,0,0,1];
G_n = C_n * ((s.*eye(5) - matVelA) \ matVelB);
% rltool(G_n);
K_n = K_n_P + K_n_I/s + K_n_D*s;


%% Latitude
% ___________________ Roll_rate __________________________________________
C_p = [0,1,0,0];
G_pr = C_p * ((s.*eye(4) - matLatA) \ matLatB);
%rltool(G_pr);
K_p   = K_p_P + K_p_I/s + K_p_D*s;

matRateA = matLatA-matLatB*K_p*C_p;

% ___________________ Roll ________________________________________________
C_phi = [0,0,1,0];
G_p = C_phi * ((s.*eye(4) - matRateA) \ matLatB);
%rltool(G_p);
K_phi = K_phi_P + K_phi_I/s + K_phi_D*s;

matTheA = matRateA-matLatB*K_phi*C_phi;

% ___________________ Lateral Velocity ____________________________________
C_v = [0,0,0,1];
G_v = C_v * ((s.*eye(4) - matTheA) \ matLatB);
%rltool(G_v);
K_v = K_v_P + K_v_I/s + K_v_D*s;

matVLatA = [matTheA-matLatB*K_v*C_v, zeros(4,1);
            0,0,0,1,0];
matVLatB = [matLatB;0];

% ___________________ Lateral Position_____________________________________
C_e = [0,0,0,0,1];
G_e = C_e * ((s.*eye(5) - matVLatA) \ matVLatB);
% rltool(G_e);
K_e = K_e_P + K_e_I/s + K_e_D*s;

%% Directional 

% __________________ yaw_rate _____________________________________________
C_r = [0, 1, 0];
G_r = C_r* ((s.*eye(3) - matDirA) \ matDirB);
% rltool(G_r);
K_r   = K_r_P + K_r_I/s + K_r_D*s;

matYawA = matDirA - matDirB*K_r*C_r;

% __________________ yaw __________________________________________________
C_psi = [0, 0, 1];
G_psi = C_psi* ((s.*eye(3) - matYawA) \ matDirB);
% rltool(G_psi);
K_psi   = K_psi_P + K_psi_I/s + K_psi_D*s;

%% Heave

% __________________ Vertical Velocity_____________________________________
C_w = [0, 1, 0];
G_w = C_w* ((s.*eye(3) - matHevA) \ matHevB);
%  rltool(G_w);
K_w   = K_w_P + K_w_I/s + K_w_D*s;
% minus PID
matZVelA = matHevA + matHevB*K_w*C_w;

% __________________ Vertical Position ____________________________________
C_d = [0, 0, 1];
G_d = C_d* ((s.*eye(3) - matZVelA) \ matHevB);
%  rltool(G_d);
K_d   = K_d_P + K_d_I/s + K_d_D*s;
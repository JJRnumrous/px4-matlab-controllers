%% Simulation
step_input = 'pitch_rate';
%step_input = 'none';

latest = 0;
log_name = "logs/pitch_rate_log.ulg";

sim_time = 30;
sim_freq = 250;

%% Constants
R_earth = 6371000; % radius of earth
g = 9.80665; % gravitational constant
eta_w = 0.2; % eta constant
rho = 1.225; % air density

lat_runway = 47.397742;
lon_runway = 8.545594;
alt_runway = 488;

R_LD = 10; % lift-to-drag ratio

%% Quad Model
quad_parameters_IARTF;

I = [Ixx 0 0 ; 0 Iyy 0 ; 0 0 Izz]; % inertia
mixin_mat = inv(virtual_controls_mat); % mixin matrix
max_T = max_total_T / 4; % maximum thrust per motor
hover_T = max_total_T * hover_perc; % hover thrust (total)

%% Quad Control
controller_gains_IARTF;

%% Read PX4 Log
read_px4_log;

%% Step Response
is_step = true; % set default value

pitch_rate_step_time = 0;
pitch_rate_step_val = 0;
roll_rate_step_time = 0;
roll_rate_step_val = 0;
yaw_rate_step_time = 0;
yaw_rate_step_val = 0;

pitch_step_time = 0;
pitch_step_val = 0;
roll_step_time = 0;
roll_step_val = 0;
yaw_step_time = 0;
yaw_step_val = 0;

vx_step_time = 0;
vx_step_val = 0;
vy_step_time = 0;
vy_step_val = 0;
vz_step_time = 0;
vz_step_val = 0;

n_step_time = 0;
n_step_val = 0;
e_step_time = 0;
e_step_val = 0;
d_step_time = 0;
d_step_val = 0;

switch step_input
    case 'roll_rate'
        time = time_att;
        time_sp = time_rates_sp;
        step = roll_rate;
        step_sp = roll_rate_sp;
    case 'pitch_rate'
        time = time_att;
        time_sp = time_rates_sp;
        step = pitch_rate;
        step_sp = pitch_rate_sp;
    case 'yaw_rate'
        time = time_att;
        time_sp = time_rates_sp;
        step = yaw_rate;
        step_sp = yaw_rate_sp;
    case 'roll'
        time = time_att;
        time_sp = time_att_sp;
        step = roll;
        step_sp = roll_sp;
    case 'pitch'
        time = time_att;
        time_sp = time_att_sp;
        step = pitch;
        step_sp = pitch_sp;
    case 'yaw'
        time = time_att;
        time_sp = time_att_sp;
        step = yaw;
        step_sp = yaw_sp;
    case 'vx'
        time = time_pos;
        time_sp = time_pos_sp;
        step = vx;
        step_sp = vx_sp;
    case 'vy'
        time = time_pos;
        time_sp = time_pos_sp;
        step = vy;
        step_sp = vy_sp;
    case 'vz'
        time = time_pos;
        time_sp = time_pos_sp;
        step = vz;
        step_sp = vz_sp;
    case 'n'
        time = time_pos;
        time_sp = time_pos_sp;
        step = north;
        step_sp = n_sp;
    case 'e'
        time = time_pos;
        time_sp = time_pos_sp;
        step = east;
        step_sp = e_sp;
    case 'd'
        time = time_pos;
        time_sp = time_pos_sp;
        step = down;
        step_sp = d_sp;
    otherwise
        is_step = false;
end

if is_step == true
    for i = 1:numel(time_sp)
        if step_sp(i) > step_sp(numel(step_sp)) / 10 % larger than 10% of step
            eval([step_input '_step_time = time_sp(i-1)']);
            break;
        end
    end
    eval([step_input '_step_val = step_sp(numel(step_sp))']);
    sim_time = time_sp(numel(time_sp));
    
    sim px4_controllers_IARTF.slx;
    sim_out = eval(['sim_' step_input]);
    plot(time_sp, step_sp, time, step, sim_out.time, sim_out.data);
    legend("step", "px4", "matlab");
end
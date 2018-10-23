%% Extract desired messages from log to CSV files
messages = ["vehicle_rates_setpoint", "vehicle_attitude_setpoint", "vehicle_local_position_setpoint", "vehicle_attitude", "vehicle_local_position"];
output_dir = "px4_logs_csv";
delimeter = ",";
ulog_filename = "px4_log";
ulog_file = strcat(ulog_filename, ".ulg");

if latest == 1
   px4_latest_log;
else
   cmd_copy = strcat("cp ", log_name, " ", output_dir, "/", ulog_file);
   system(cmd_copy);
end

messages_str = messages(1);
for i = 2:5
   messages_str = strcat(messages_str, ",", messages(i)); 
end
csv_command = strcat("ulog2csv -m ", messages_str, " -o ", output_dir, " -d ", delimeter, " ", output_dir, "/", ulog_file);
system(csv_command);

%% Read setpoints
rates_sp_msg_name = messages(1);
rates_sp_csv = csvread(strcat(output_dir, "/", ulog_filename, "_", rates_sp_msg_name, "_0.csv"), 1, 0);

att_sp_msg_name = messages(2);
att_sp_csv = csvread(strcat(output_dir, "/", ulog_filename, "_", att_sp_msg_name, "_0.csv"), 1, 0);

pos_sp_msg_name = messages(3);
pos_sp_csv = csvread(strcat(output_dir, "/", ulog_filename, "_", pos_sp_msg_name, "_0.csv"), 1, 0);

%% Read states (attitude and position)
att_msg_name = messages(4);
att_csv = csvread(strcat(output_dir, "/", ulog_filename, "_", att_msg_name, "_0.csv"), 1, 0);

pos_msg_name = messages(5);
pos_csv = csvread(strcat(output_dir, "/", ulog_filename, "_", pos_msg_name, "_0.csv"), 1, 0);

%% Get fields of interest
time_rates_sp = rates_sp_csv(:,1) ./ 1e6;
roll_rate_sp = rates_sp_csv(:,2);
pitch_rate_sp = rates_sp_csv(:,3);
yaw_rate_sp = rates_sp_csv(:,4);

time_att_sp = att_sp_csv(:,1) ./ 1e6;
roll_sp = att_sp_csv(:,2);
pitch_sp = att_sp_csv(:,3);
yaw_sp = att_sp_csv(:,4);

time_att = att_csv(:,1) ./ 1e6;
roll_rate = att_csv(:,2);
pitch_rate = att_csv(:,3);
yaw_rate = att_csv(:,4);
q = [att_csv(:,5) att_csv(:,6) att_csv(:,7) att_csv(:,8)];
eul = quat2eul(q);
roll = eul(:,1);
pitch = eul(:,2);
yaw = eul(:,3);

time_pos_sp = pos_sp_csv(:,1) ./ 1e6;
vx_sp = pos_sp_csv(:,7);
vy_sp = pos_sp_csv(:,8);
vz_sp = pos_sp_csv(:,9);
n_sp = pos_sp_csv(:,2);
e_sp = pos_sp_csv(:,3);
d_sp = pos_sp_csv(:,4);

time_pos = pos_csv(:,1) ./ 1e6;
vx = pos_csv(:,11);
vy = pos_csv(:,12);
vz = pos_csv(:,13);
north = pos_csv(:,5);
east = pos_csv(:,6);
down = pos_csv(:,7);

function eul = quat2eul(q)
    eul(:,1) = atan2(2.*(q(:,1).*q(:,2)+q(:,3).*q(:,4)), 1-2.*(q(:,2).^2+q(:,3).^2));
    eul(:,2) = asin(2.*(q(:,1).*q(:,3)-q(:,4).*q(:,2)));
    eul(:,3) = atan2(2.*(q(:,1).*q(:,4)+q(:,2).*q(:,3)), 1-2.*(q(:,3).^2+q(:,4).^2));
end
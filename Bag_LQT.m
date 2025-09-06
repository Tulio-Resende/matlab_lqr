close all
clear all

% Select the bag (the path where the bag is)
% bag = rosbag('/home/tulio/volumes/bags/LQT/');
bag = rosbag('/home/tulio/matlab_scripts/LQR/bags/LQT_landingPadON.bag');


% See the available topics saved in the record
% e melhor lebag.AvailableTopics


% Select the topic/topics you'd like to analyse (filter of topics)
ref = select(bag, 'Topic', '/rl_control/output_reference');
vel = select(bag, 'Topic', '/dji_sdk/velocity');
pos = select(bag, 'Topic', '/dji_sdk/local_position');
control_effort = select(bag, 'Topic', '/dji_sdk/flight_control_setpoint_generic');

% Save messages as structs
msg_ref = readMessages(ref, 'DataFormat', 'struct');
msg_vel = readMessages(vel, 'DataFormat', 'struct');
msg_pos = readMessages(pos, 'DataFormat', 'struct');
msg_control = readMessages(control_effort, 'DataFormat', 'struct');

time_ref = ref.MessageList.Time - ref.StartTime;
time_vel = vel.MessageList.Time - vel.StartTime;
time_pos = pos.MessageList.Time - pos.StartTime;
time_control = control_effort.MessageList.Time - control_effort.StartTime;

n_ref = length(msg_ref);
ref_model = zeros(n_ref,5);

for i = 1:n_ref
    ref_model(i,1) = msg_ref{i}.X;
    ref_model(i,2) = msg_ref{i}.Y;
end

n_pos = length(msg_pos);
pos_drone = zeros(n_pos,5);

for i = 1:n_pos
    pos_drone(i,1) = msg_pos{i}.Point.X;
    pos_drone(i,2) = msg_pos{i}.Point.Y;
end

n_control = length(msg_control);
control_drone = zeros(n_control,2);

for i = 1:n_control
    control_drone(i,1) = msg_control{i}.Axes(1);
    control_drone(i,2) = msg_control{i}.Axes(2);
end


figure;
plot(ref_model(:,1), ref_model(:,2), 'LineWidth', 1.5);
grid on;
hold on
plot(pos_drone(:,1), pos_drone(:,2), 'LineWidth', 1.5);

figure(2)
plot(time_ref(:,1), ref_model(:,1), 'LineWidth', 1.5);
grid on;
hold on
plot(time_pos(2:end, 1), pos_drone(2:end,1), 'LineWidth', 1.5);

figure(3)
plot(time_ref(:,1), ref_model(:,2), 'LineWidth', 1.5);
grid on;
hold on
plot(time_pos(2:end, 1), pos_drone(2:end,2), 'LineWidth', 1.5);

figure(4)
plot(time_control(:,1), control_drone(:,1), 'LineWidth', 1.5);
grid on;
hold on
plot(time_control(:,1), control_drone(:,2), 'LineWidth', 1.5);
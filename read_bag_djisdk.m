clear all
close all

a=1;
b=1;

freq = 50;
h=1/freq;

bag = rosbag("2ndbag.bag");

ym_topic = select(bag, "Topic", "/desired/position");

u_topic = select(bag, "Topic", "/dji_sdk/flight_control_setpoint_generic");

y_topic = select(bag, "Topic", "/dji_sdk/local_position");
dy_topic = select(bag, "Topic", "/dji_sdk/velocity");
ddy_topic = select(bag, "Topic", "/dji_sdk/acceleration_ground_fused");

time_y = y_topic.MessageList.Time;
time_dy = dy_topic.MessageList.Time;
time_ddy = ddy_topic.MessageList.Time;
time_ym = ym_topic.MessageList.Time;
time_u = u_topic.MessageList.Time;

t0 = double(max([time_u(1), time_y(1), time_ym(1)]));
tf = double(min([time_u(end), time_y(end), time_ym(end)]));

t = [0:h:tf-t0];

% time_y = time_y - t0;
% time_dy = time_dy - t0;
% time_ddy = time_ddy - t0;
% time_ym = time_ym - t0;
% time_u = time_u - t0;

msgs_ym = readMessages(ym_topic, "DataFormat", "struct");
msgs_u = readMessages(u_topic, "DataFormat", "struct");
msgs_y = readMessages(y_topic, "DataFormat", "struct");
msgs_dy = readMessages(dy_topic, "DataFormat", "struct");
msgs_ddy = readMessages(ddy_topic, "DataFormat", "struct");

ym = cellfun(@(m) double(m.Vector.X), msgs_ym);
u = cellfun(@(m) double(m.Axes(1)), msgs_u);
y = cellfun(@(m) double(m.Point.X), msgs_y);
dy = cellfun(@(m) double(m.Vector.X), msgs_dy);
ddy = cellfun(@(m) double(m.Vector.X), msgs_ddy);

tin = time_y - t0;
y_interp = interp1(tin, y, time_u, 'linear', 'extrap');

tin = time_dy - t0;
dy_interp = interp1(tin, dy, time_u, 'linear', 'extrap');

tin = time_ddy - t0;
ddy_interp = interp1(tin, ddy, time_u, 'linear', 'extrap');

tin = time_ym - t0;
ym_interp = interp1(tin, ym, time_u, 'linear', 'extrap');

figure(1);
plot(t, ym_interp);
hold on
plot(t, y_interp, 'r');
grid on
hold off
legend('$y_m$', '$y$', 'Interpreter', 'latex');

%% Estimate wind disturbance
ddy_hat = diff(dy_interp)/h;

n_pts = length(y_interp);

d_hat = (ddy_hat(1:n_pts) + a*dy_interp(1:n_pts))/b - u;

figure(2);
plot(t, d_hat, 'b');
hold on
%plot(t, u, 'r')
%plot(t, ddy_interp, 'k');
grid on
%hold off
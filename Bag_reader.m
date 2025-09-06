close all
clear all

% Select the bag (the path where the bag is)
bag = rosbag('/home/tulio/matlab_scripts/LQR/bags/test1.bag');



% See the available topics saved in the record
% bag.AvailableTopics


% Select the topic/topics you'd like to analyse (filter of topics)
% ref_generator = select(bag, 'Topic', '/rl_control/ref_generator');

% Save messages as structs
msgs = readMessages(bag, 'DataFormat', 'struct');


time = bag.MessageList.Time - bag.StartTime;

n = length(msgs);
ref_states = zeros(n,5);

for i = 1:n
    ref_states(i,1) = msgs{i}.Data(1);
    ref_states(i,2) = msgs{i}.Data(2);
    ref_states(i,3) = msgs{i}.Data(3);
    ref_states(i,4) = msgs{i}.Data(4);
    ref_states(i,5) = msgs{i}.Data(5);
end

figure;
plot(time, ref_states(:,5), 'LineWidth', 1.5);
xlabel('Tempo [s]');
ylabel('Posição X [m]');
title('Posição X vs Tempo');
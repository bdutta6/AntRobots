% data = csvread('pid_test_1.csv'); %kp = 0.5
% time = data(:,1)./1000;
% set_pos = data(:,2);
% act_pos = data(:,3);
% error = data(:,4);
% % dT = diff(time);
% % dset_pos = diff(set_pos)./dT;
% hold on
% plot(time,set_pos,'--b',time,act_pos,'-r'); %,time, error, '-k')

% data = csvread('pid_test_2.csv'); %kp = 0.75
% time = data(:,1)./1000;
% set_pos = data(:,2);
% act_pos = data(:,3);
% error = data(:,4);
% % dT = diff(time);
% % dset_pos = diff(set_pos)./dT;
% hold on
% plot(time,set_pos,'--b',time,act_pos,'-g'); %,time, error, '-k')

% data = csvread('pid_test_3.csv'); %kp = 1
% time = data(:,1)./1000;
% set_pos = data(:,2);
% act_pos = data(:,3);
% error = data(:,4);
% % dT = diff(time);
% % dset_pos = diff(set_pos)./dT;
% hold on
% plot(time,set_pos,'--b',time,act_pos,'-k'); %,time, error, '-k')

% data = csvread('pid_test_4.csv'); %kp = 1.5
% time = data(:,1)./1000;
% set_pos = data(:,2);
% act_pos = data(:,3);
% error = data(:,4);
% % dT = diff(time);
% % dset_pos = diff(set_pos)./dT;
% hold on
% plot(time,set_pos,'--b',time,act_pos,'-r'); %,time, error, '-k')

% data = csvread('pid_test_5.csv'); %kp = 1.25
% time = data(:,1)./1000;
% set_pos = data(:,2);
% act_pos = data(:,3);
% error = data(:,4);
% % dT = diff(time);
% % dset_pos = diff(set_pos)./dT;
% hold on
% plot(time,set_pos,'--b',time,act_pos,'-r'); %,time, error, '-k')

subplot(3,1,1)
data = csvread('pid_test_6.csv'); %kp = 1.25 setpoint = 1
time = data(:,1)./1000;
set_pos = data(:,2);
act_pos = data(:,3);
error = data(:,4);
% dT = diff(time);
% dset_pos = diff(set_pos)./dT;
hold on
% plot(time,set_pos,'--b',time,act_pos,'-r'); %,time, error, '-k')
plot(time,error,'-r')
hold off 

subplot(3,1,2)
data = csvread('pid_test_8.csv'); %kp = 1.125 setpoint = 1
time = data(:,1)./1000;
set_pos = data(:,2);
act_pos = data(:,3);
error = data(:,4);
% dT = diff(time);
% dset_pos = diff(set_pos)./dT;
hold on
% plot(time,set_pos,'--b',time,act_pos,'-r'); %,time, error, '-k')
plot(time,error,'-b')
hold off

subplot(3,1,3)
data = csvread('pid_test_9.csv'); %kp = 1.0625 setpoint = 1
time = data(:,1)./1000;
set_pos = data(:,2);
act_pos = data(:,3);
error = data(:,4);
% dT = diff(time);
% dset_pos = diff(set_pos)./dT;
hold on
% plot(time,set_pos,'--b',time,act_pos,'-r'); %,time, error, '-k')
plot(time,error,'-g')
hold off
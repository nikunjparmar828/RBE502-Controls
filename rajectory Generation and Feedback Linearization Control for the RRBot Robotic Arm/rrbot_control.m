clear; close; clc;

syms qd1 qd2 t;

rosinit;

j1_effort = rospublisher('/rrbot/joint1_effort_controller/command');
j2_effort = rospublisher('/rrbot/joint2_effort_controller/command');
JointStates = rossubscriber('/rrbot/joint_states');
tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j2_effort);
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'rrbot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint2'};
req.JointPositions = [deg2rad(30), deg2rad(45)];
resp = call(client,req,'Timeout',3);

%Trajctory data---
[qd1;qd2] = [180-(27/5)*t^2 + (9/25)*t^3; 90 - (2.7)*t^2 + (9/50)*t^3];
%-------------------

tic;
t = 0;
dX = zeros(6,10);
ii = 1; %counter

while(t < 10)

t = toc;
% read the joint states
jointData = receive(JointStates);
% inspect the "jointData" variable in MATLAB to get familiar with its structure
% design your state feedback controller in the following

K = [25.4537, 7.6688, 6.4562, 2.1219; 6.4544, 5.6456, 1.9244, 0.8198];
F = -K*[wrapToPi(jointData.Position(1));wrapToPi(jointData.Position(2));jointData.Velocity(1);jointData.Velocity(2)];

tau1.Data = F(1);
tau2.Data = F(2);
send(j1_effort,tau1);
send(j2_effort,tau2);
dX(1,ii) = wrapToPi(jointData.Position(1));
dX(2,ii) = wrapToPi(jointData.Position(2));
dX(3,ii) = jointData.Velocity(1);
dX(4,ii) = jointData.Velocity(2);
dX(5,ii) = F(1);
dX(6,ii) = F(2);
ii = ii +1;

% you can sample data here to be plotted at the end

end

tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
% disconnect from roscore
rosshutdown;


t = linspace(0,10,ii-1);

figure;

subplot(3,3,1);
plot(t,rad2deg(dX(1,:)),'r');
xlabel('t','FontSize',12);
ylabel('theta_1','FontSize',12);

subplot(3,3,2);
plot(t,rad2deg(dX(2,:)),'g');
xlabel('t','FontSize',12);
ylabel('theta_2','FontSize',12);

subplot(3,3,3);
plot(t,dX(3,:),'b');
xlabel('t','FontSize',12);
ylabel('theta_dot_1','FontSize',12);

subplot(3,3,4);
plot(t,dX(4,:),'r');
xlabel('t','FontSize',12);
ylabel('theta_dot_2','FontSize',12);

subplot(3,3,5);
plot(t,dX(5,:),'g');
xlabel('t','FontSize',12);
ylabel('t1','FontSize',12);

subplot(3,3,6);
plot(t,dX(6,:),'b');
xlabel('t','FontSize',12);
ylabel('t2','FontSize',12);
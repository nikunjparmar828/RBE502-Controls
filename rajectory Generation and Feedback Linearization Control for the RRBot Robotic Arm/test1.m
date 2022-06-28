
clear; close; clc;
% ROS Setup
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
req.JointPositions = [deg2rad(200), deg2rad(125)];
resp = call(client,req,'Timeout',3);
tic;
t = 0;
count = 1;
dX = zeros(6,10);
x_desired = zeros(4,10);
while(t < 10)
t = toc;
% read the joint states

jointData = receive(JointStates);

% inspect the "jointData" variable in MATLAB to get familiar with its structure
% design your state feedback controller in the following
theta1 = (jointData.Position(1));
theta2 = (jointData.Position(2));
theta1_dot = jointData.Velocity(1);
theta2_dot = jointData.Velocity(2);

%Manipubality form
M = [(9*cos(theta2))/10 + 1573/1000, (9*cos(theta2))/20 + 573/2000;
 (9*cos(theta2))/20 + 573/2000,                      573/2000];
C = [-(9*theta2_dot*sin(theta2)*(2*theta1_dot + theta2_dot))/20;
                           (9*theta1_dot^2*sin(theta2))/20];
G = [- (8829*sin(theta1 + theta2))/2000 - (28449*sin(theta1))/2000;
                            -(8829*sin(theta1 + theta2))/2000];
%Trajectory generation
theta1_desired = ((pi*t^3)/500 - (3*pi*t^2)/100 + pi);
theta1d_desired = (3*pi*t^2)/500 - (3*pi*t)/50;
theta1dd_desired = (3*pi*t*2)/500 - (3*pi)/50;
theta2_desired = ((pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2);
theta2d_desired = (3*pi*t^2)/1000 - (3*pi*t)/100;
theta2dd_desired = (3*pi*t*2)/1000 - (3*pi)/100;
%virtual control law
K = [70.5017   -6.0063   12.8314   -1.3136;
   -4.2249   73.9042    0.4738   13.1686];
v = -K*[theta1-theta1_desired;theta2-theta2_desired;theta1_dot-theta1d_desired;theta2_dot-theta2d_desired]+[theta1dd_desired;theta2dd_desired];
%Control Law 
F = M*v + C + G;
tau1.Data = F(1);
tau2.Data = F(2);

send(j1_effort,tau1);
send(j2_effort,tau2);
dX(1,count) = (jointData.Position(1));
dX(2,count) = (jointData.Position(2));
dX(3,count) = jointData.Velocity(1);
dX(4,count) = jointData.Velocity(2);
dX(5,count) = F(1);
dX(6,count) = F(2);
x_desired(1,count) = theta1_desired;
x_desired(2,count) = theta2_desired;
x_desired(3,count) = theta1d_desired;
x_desired(4,count) = theta2d_desired;
count = count +1;
% you can sample data here to be plotted at the end
end
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
% disconnect from roscore
rosshutdown;

t = linspace(0,10,count-1);

figure;
subplot(3,2,1);
plot(t,rad2deg(dX(1,:)),'b');
hold on
plot(t,rad2deg(x_desired(1,:)),'r');
xlabel('t','FontSize',14);
ylabel('theta1','FontSize',14);

subplot(3,2,2);
plot(t,rad2deg(dX(2,:)),'b');
hold on
plot(t,rad2deg(x_desired(2,:)),'r');
xlabel('t','FontSize',14);
ylabel('theta2','FontSize',14);

subplot(3,2,3);
plot(t,dX(3,:),'b');
hold on
plot(t,(x_desired(3,:)),'r');
xlabel('t','FontSize',14);
ylabel('theta1_dot','FontSize',14);

subplot(3,2,4);
plot(t,dX(4,:),'b');
hold on
plot(t,(x_desired(4,:)),'r');
xlabel('t','FontSize',14);
ylabel('theta2_dot','FontSize',14);

subplot(3,2,5);
plot(t,dX(5,:),'b');
xlabel('t','FontSize',14);
ylabel('torque1','FontSize',14);

subplot(3,2,6);
plot(t,dX(6,:),'b');
xlabel('t','FontSize',14);
ylabel('torque2','FontSize',14);
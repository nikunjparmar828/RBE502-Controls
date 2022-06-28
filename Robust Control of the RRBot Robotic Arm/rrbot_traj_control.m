clear; close; clc;

syms theta_1d theta_2d theta_dot_1d theta_dot_2d theta_ddot_1d theta_ddot_2d t;
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
dX = zeros(6,10);
x_desired = zeros(4,10);
ii = 1; %counter

while(t < 10)

t = toc;
% read the joint states
jointData = receive(JointStates);
% inspect the "jointData" variable in MATLAB to get familiar with its structure
% design your state feedback controller in the following
theta_1d = pi*(1-0.03.*t.^2+0.002.*t.^3);
theta_2d = (pi/2)*(1-0.03.*t.^2+0.002.*t.^3);
theta_dot_1d = pi*(-0.06.*t+0.006.*t.^2);
theta_dot_2d = (pi/2)*(-0.06.*t+0.006.*t.^2);
theta_ddot_1d = pi*(-0.06+0.012.*t);
theta_ddot_2d = (pi/2)*(-0.06+0.012.*t);

%-----------------------------------------
K = [69.45   -6.103   11.993   -1.3136;
   -4.1249   74.000    0.508   12.96];

%----------------------------------------
Rho = 1.2;
Phi = 0.005;
A = [0 0 1 0;0 0 0 1;0 0 0 0;0 0 0 0]; 
B = [0 0;0 0;1 0;0 1];

K = [69.45   -6.103   11.993   -1.3136;
   -4.1249   74.000    0.508   12.96];

Acl = A-B*K;

Q = eye(4).*10;
P = lyap(Acl',Q);

error= [jointData.Position(1); jointData.Position(2); jointData.Velocity(1); jointData.Velocity(2)]-[theta_1d; theta_2d; theta_dot_1d; theta_dot_2d]; 

if Phi >0
    if norm(B'*P*error) ~=0 
        Vr = (-Rho*(B'*P*error))/norm(B'*P*error);
    else
        Vr = (-Rho*(B'*P*error))/Phi;
    end
else
    if norm(B'*P*error) > Phi
        Vr = (-Rho*(B'*P*error))/norm(B'*P*error);
    else
        Vr = 0;
    end 
end
    

v = -K*(error) + [theta_ddot_1d;theta_ddot_2d] + Vr;

%F = [v(1)*((9*cos(theta_2))/10+1573/1000)+v(2)*((9*cos(theta_2))/20+573/2000);(573*v(2))/2000+v(1)*((9*cos(theta_2))/20+573/2000)] + [-(theta_dot_2*((9*sin(theta_2)*(theta_dot_1+theta_dot_2))/10+(9*theta_dot_1*sin(theta_2))/10))/2;(9*theta_dot_1*sin(theta_2)*(theta_dot_1+theta_dot_2))/20-(9*theta_dot_1*theta_dot_2*sin(theta_2))/20] + [-(8829*sin(theta_1+theta_2))/2000-(28449*sin(theta_1))/2000;-(8829*sin(theta_1+theta_2))/2000];

F = [v(1)*((27*cos(jointData.Position(2)))/40 + 4719/4000) - (85347*sin(jointData.Position(1)))/8000 - (26487*sin(jointData.Position(1) + jointData.Position(2)))/8000 + v(2)*((27*cos(jointData.Position(2)))/80 + 1719/8000) - (3*jointData.Velocity(2)*((9*sin(jointData.Position(2))*(jointData.Velocity(1) + jointData.Velocity(2)))/10 + (9*jointData.Velocity(1)*sin(jointData.Position(2)))/10))/8;
                                             (1719*v(2))/8000 - (26487*sin(jointData.Position(1) + jointData.Position(2)))/8000 + v(1)*((27*cos(jointData.Position(2)))/80 + 1719/8000) + (27*jointData.Velocity(1)*sin(jointData.Position(2))*(jointData.Velocity(1) + jointData.Velocity(2)))/80 - (27*jointData.Velocity(1)*jointData.Velocity(2)*sin(jointData.Position(2)))/80];

tau1.Data = F(1);
tau2.Data = F(2);
send(j1_effort,tau1);
send(j2_effort,tau2);
dX(1,ii) = jointData.Position(1);
dX(2,ii) = jointData.Position(2);
dX(3,ii) = jointData.Velocity(1);
dX(4,ii) = jointData.Velocity(2);
dX(5,ii) = F(1);
dX(6,ii) = F(2);
x_desired(1,ii) = theta_1d;
x_desired(2,ii) = theta_2d;
x_desired(3,ii) = theta_dot_1d;
x_desired(4,ii) = theta_dot_2d;
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
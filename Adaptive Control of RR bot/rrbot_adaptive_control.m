clear; close; clc;
% Nikunj
% syms theta_1d theta_2d theta_dot_1d theta_dot_2d theta_ddot_1d theta_ddot_2d t;
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


m1 = 1; m2 = 1; g=9.81; l1 = 1; l2 = 1; r1 = 0.45; r2 = 0.45; 
I1 = 0.084; I2 = 0.084;

A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];

lambda = [-3 -3 -4 -4];
K = place(A, B, lambda);

Acl = A - B*K;
Q = eye(4);
P = lyap(Acl',Q);

tic;
t = 0;
i = 1; %counter

% alpha values from the pdf 
alpha = [m2*l1^2 + m1*r1^2 + m2*r2^2 + I1 + I2;
m2*l1*r2
m2*r2^2 + I2
m1*r1 + m2*l1
m2*r2];

new_alpha = 0.75*alpha;

alpha1(i) = new_alpha(1);
alpha2(i) = new_alpha(2);
alpha3(i) = new_alpha(3);
alpha4(i) = new_alpha(4);
alpha5(i) = new_alpha(5);


while(t < 10)
    t = toc;
    
    % read the joint states
    jointData = receive(JointStates);

    M_hat= [alpha1(i)+2*alpha2(i)*cos(jointData.Position(2)), alpha3(i)+alpha2(i)*cos(jointData.Position(2)); alpha3(i)+alpha2(i)*cos(jointData.Position(2)), alpha3(i)];
    C_hat= [-alpha2(i)*sin(jointData.Position(2))*jointData.Velocity(2), -alpha2(i)*sin(jointData.Position(2))*(jointData.Velocity(1)+jointData.Velocity(2)); alpha2(i)*sin(jointData.Position(2))*jointData.Velocity(1),0];
    G_hat= [-alpha4(i)*g*sin(jointData.Position(1))-alpha5(i)*g*sin(jointData.Position(1)+jointData.Position(2)); -alpha5(i)*g*sin(jointData.Position(1)+jointData.Position(2))];
    

    htheta_1d(i) = deg2rad(180)*(1-0.03*t^2+0.002*t^3);
    htheta_2d(i) = deg2rad(90)*(1-0.03*t^2+0.002*t^3);
    htheta_dot_1d(i) = deg2rad(180)*(-0.06*t+0.006*t^2);
    htheta_dot_2d(i) = deg2rad(90)*(-0.06*t+0.006*t^2);
    htheta_ddot_1d(i) = deg2rad(180)*(-0.06+0.012*t);
    htheta_ddot_2d(i) = deg2rad(90)*(-0.06+0.012*t);


    e = [jointData.Position(1); jointData.Position(2)] - [htheta_1d(i); htheta_2d(i)];

    e_dot = [jointData.Velocity(1); jointData.Velocity(2)] - [htheta_dot_1d(i); htheta_dot_2d(i)];

    error = [e; e_dot];

    v = [htheta_ddot_1d(i); htheta_ddot_2d(i)] - K*error;

    U = M_hat * v + C_hat * [jointData.Velocity(1); jointData.Velocity(2)] + G_hat;
   
    tau1.Data = U(1);
    tau2.Data = U(2);

    send(j1_effort,tau1);
    send(j2_effort,tau2);
   
    M = [I1+I2+m1*r1^2+m2*r2^2+m2*l1^2+2*m2*l1*r2*cos(jointData.Position(2))    I2+m2*r2^2+m2*r2*l1*cos(jointData.Position(2));
        I2+m2*r2^2+m2*r2*l1*cos(jointData.Position(2))                         I2+m2*r2^2];

    C = [0                          -m2*l1*r2*sin(jointData.Position(2))*(2*jointData.Velocity(1)+jointData.Velocity(2));
        m2*l1*r2*sin(jointData.Position(2))*jointData.Velocity(1)      0];  

    G = [-m2*g*r2*sin(jointData.Position(1)+jointData.Position(2))-g*l1*m2*sin(jointData.Position(1))-g*m1*r1*sin(jointData.Position(1));
        -g*m2*r2*sin(jointData.Position(1)+jointData.Position(2))];

    theta_ddot = M\(U - C*[jointData.Velocity(1); jointData.Velocity(2)] - G);

    Y = [theta_ddot(1), ...
    cos(jointData.Position(2))*(2*theta_ddot(1) + theta_ddot(2)) - 2*sin(jointData.Position(2))*jointData.Velocity(1)*jointData.Velocity(2) - sin(jointData.Position(2))*jointData.Velocity(2)^2, ...
    theta_ddot(2), ...
    -sin(jointData.Position(1))*g, ...
    -sin(jointData.Position(1) + jointData.Position(2))*g; ...
    0, ...
    sin(jointData.Position(2))*jointData.Velocity(1)^2 + cos(jointData.Position(2))*theta_ddot(1), ...
    theta_ddot(1) + theta_ddot(2), ...
    0, ...
    -sin(jointData.Position(1)+jointData.Position(2))*g];

    phi = M_hat\Y;

    gamma = eye(5)*40;
    
    est_alpha_dot = -gamma\(phi'*B'*P*error);
    
    if i == 1
        dt = t;
    else
        dt = t - time(i-1);
    end
    
    alpha1(i+1) = alpha1(i) + (est_alpha_dot(1)*dt);
    alpha2(i+1) = alpha2(i) + (est_alpha_dot(2)*dt);
    alpha3(i+1) = alpha3(i) + (est_alpha_dot(3)*dt);
    alpha4(i+1) = alpha4(i) + (est_alpha_dot(4)*dt);
    alpha5(i+1) = alpha5(i) + (est_alpha_dot(5)*dt);
   

    dX(1,i) = jointData.Position(1);
    dX(2,i) = jointData.Position(2);
    dX(3,i) = jointData.Velocity(1);
    dX(4,i) = jointData.Velocity(2);
    dX(5,i) = tau1.Data;
    dX(6,i) = tau2.Data;
%     time(i) = t;
%     
%     x_desired(1,i) = theta_1d;
%     x_desired(2,i) = theta_2d;
%     x_desired(3,i) = theta_dot_1d;
%     x_desired(4,i) = theta_dot_2d;

    time(i)=t;
    i=i+1;  
   
end


tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
% disconnect from roscore
rosshutdown;



figure;
subplot(3,3,1);
plot(time,rad2deg(dX(1,:)));
hold on
plot(time,rad2deg(htheta_1d));
xlabel('t');
ylabel('theta 1');

subplot(3,3,2);
plot(time,rad2deg(dX(2,:)));
hold on
plot(time,rad2deg(htheta_2d));
xlabel('t');
ylabel('theta 2');

subplot(3,3,3);
plot(time,rad2deg(dX(3,:)));
hold on
plot(time,rad2deg(htheta_dot_1d));
xlabel('t');
ylabel('theta 1 dot');

subplot(3,3,4);
plot(time,rad2deg(dX(4,:)));
hold on 
plot(time,rad2deg(htheta_dot_2d));
xlabel('t');
ylabel('theta 2 dot');

subplot(3,3,5);
plot(time,dX(5,:));
xlabel('t');
ylabel('u1');

subplot(3,3,6);
plot(time,dX(6,:));
xlabel('t');
ylabel('u2')
 

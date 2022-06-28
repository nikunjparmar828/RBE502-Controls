% Nikunj Parmar
clc; close all; clear;

tspan = [0,10];

syms theta_1 theta_2 theta_dot_1 theta_dot_2 theta_ddot_1 theta_ddot_2 dl_dtheta1 dl_dtheta2 dl_dtheta1_dot dl_dtheta2_dot ddl_dtheta1_dot_dt ddl_dtheta2_dot_dt 'real';
syms r1 r2 l1 l2 m1 m2 I1 I2 t1 t2 g K P 'real';
syms eq1 eq2;
syms x1 x2 x3 x4 'real';
syms m1_hat m2_hat I1_hat I2_hat 'real';
u = sym('u',[2,1]);

% Values initialization 
m1 = 1; m2 = 1; g=9.81; l1 = 1; l2 = 1; r1 = 0.45; r2 = 0.45; 
I1 = 0.084; I2 = 0.084;

A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];

lambda = [-3 -3 -4 -4];
K = place(A, B, lambda);

Acl = A - B*K;
Q = eye(4).*10;
P = lyap(Acl',Q);
% P = 0;

% alpha values from the pdf 
alpha = [m2*l1^2 + m1*r1^2 + m2*r2^2 + I1 + I2;
m2*l1*r2;
m2*r2^2 + I2;
m1*r1 + m2*l1;
m2*r2];

new_alpha = 0.75*alpha;

x0 = [deg2rad(200),deg2rad(125),0,0,new_alpha(1) ,new_alpha(2), new_alpha(3), new_alpha(4), new_alpha(5)];

[t,y] = ode45(@adaptive,tspan,x0);

%% Desired Trajectory --
ptheta_1d = deg2rad(180)*(1-0.03*t.^2+0.002*t.^3);
ptheta_2d = deg2rad(90)*(1-0.03*t.^2+0.002*t.^3);
ptheta_dot_1d = deg2rad(180)*(-0.06*t+0.006*t.^2);
ptheta_dot_2d = deg2rad(90)*(-0.06*t+0.006*t.^2);
ptheta_ddot_1d = deg2rad(180)*(-0.06+0.012*t);
ptheta_ddot_2d = deg2rad(90)*(-0.06+0.012*t);

for i=1:size(y,1)
    
    M_hat= [y(i, 5)+2*y(i, 6)*cos(y(i, 2)), y(i, 7)+y(i, 6)*cos(y(i, 2)); y(i, 7)+y(i, 6)*cos(y(i, 2)), y(i, 7)];
    C_hat= [-y(i, 6)*sin(y(i, 2))*y(i, 4), -y(i, 6)*sin(y(i, 2))*(y(i, 3)+y(i, 4)); y(i, 6)*sin(y(i, 2))*y(i, 3),0];
    G_hat= [-y(i, 8)*g*sin(y(i, 1))-y(i, 9)*g*sin(y(i, 1)+y(i, 2)); -y(i, 9)*g*sin(y(i, 1)+y(i, 2))];

    % Desired Trajectory --
    theta_1d = deg2rad(180)*(1-0.03*t(i)^2+0.002*t(i)^3);
    theta_2d = deg2rad(90)*(1-0.03*t(i)^2+0.002*t(i)^3);
    theta_dot_1d = deg2rad(180)*(-0.06*t(i)+0.006*t(i)^2);
    theta_dot_2d = deg2rad(90)*(-0.06*t(i)+0.006*t(i)^2);
    theta_ddot_1d = deg2rad(180)*(-0.06+0.012*t(i));
    theta_ddot_2d = deg2rad(90)*(-0.06+0.012*t(i));

    error = [y(i,1); y(i,2)] - [theta_1d; theta_2d];
    error_dot = [y(i,3); y(i,4)] - [theta_dot_1d; theta_dot_2d];
    
    v = [theta_ddot_1d;theta_ddot_2d]- K*[error; error_dot];
    
    u = M_hat * v + C_hat * [y(i,3); y(i,4)] + G_hat;
    u1(i) = double(u(1)); u2(i) = double(u(2));

end

%% plots ------------------

figure;
subplot(3,3,1);
plot(t,rad2deg(y(:,1)),'LineWidth',2);
hold on;
plot(t,rad2deg(ptheta_1d),'r','LineWidth',2.2);
xlabel('t','FontSize',12);
ylabel('theta1','FontSize',12);

subplot(3,3,2);
plot(t,rad2deg(y(:,2)),'g');
hold on;
plot(t,rad2deg(ptheta_2d),'r','LineWidth',2.2);
xlabel('t','FontSize',12);
ylabel('theta2','FontSize',12);

subplot(3,3,3);
plot(t,y(:,3),'b');
hold on;
plot(t,ptheta_dot_1d,'r','LineWidth',2.2);
xlabel('t','FontSize',12);
ylabel('theta dot 1','FontSize',12);

subplot(3,3,4);
plot(t,y(:,4),'r');
hold on;
plot(t,ptheta_dot_2d,'r','LineWidth',2.2);
xlabel('t','FontSize',12);
ylabel('theta dot 2','FontSize',12);

subplot(3,3,5);
plot(t,u1,'g');
xlabel('t','FontSize',12);
ylabel('u1','FontSize',12);

subplot(3,3,6);
plot(t,u2,'b');
xlabel('t','FontSize',12);
ylabel('u2','FontSize',12);

% adapting graphs 

figure;
subplot(5,1,1);
plot(t,y(:,5),'r');
hold on;
plot(t,alpha(1)* ones(1, length(t)));
xlabel('t');
ylabel('alpha 1');

subplot(5,1,2);
plot(t,y(:,6), 'r');
hold on;
plot(t,alpha(2)* ones(1, length(t)), 'b');
xlabel('t');
ylabel('alpha 2');

subplot(5,1,3);
plot(t,y(:,7), 'r');
hold on;
plot(t,alpha(3)* ones(1, length(t)), 'b');
xlabel('t');
ylabel('alpha 3');

subplot(5,1,4);
plot(t,y(:,8),'r');
hold on;
plot(t,alpha(4)* ones(1, length(t)), 'b');
xlabel('t');
ylabel('alpha 4');

subplot(5,1,5);
plot(t,y(:,9), 'r');
hold on;
plot(t,alpha(5)* ones(1, length(t)), 'b');
xlabel('t');
ylabel('alpha 5');


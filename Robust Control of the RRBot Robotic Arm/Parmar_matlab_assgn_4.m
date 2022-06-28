% Nikunj Parmar
clc; close all; clear;

tspan = [0 10];

syms theta_1 theta_2 theta_dot_1 theta_dot_2 theta_ddot_1 theta_ddot_2 dl_dtheta1 dl_dtheta2 dl_dtheta1_dot dl_dtheta2_dot ddl_dtheta1_dot_dt ddl_dtheta2_dot_dt 'real';
syms r1 r2 l1 l2 m1 m2 I1 I2 t1 t2 g K P 'real';
syms eq1 eq2;
syms x1 x2 x3 x4 'real';
syms m1_hat m2_hat I1_hat I2_hat 'real';


A = [0 0 1 0;0 0 0 1;0 0 0 0;0 0 0 0]; 
B = [0 0;0 0;1 0;0 1];

K = [69.45   -6.103   11.993   -1.3136;
   -4.1249   74.000    0.508   12.96];

Acl = A-B*K;

Q = eye(4).*10;
P = lyap(Acl',Q);


x0 = [deg2rad(200);deg2rad(125);0; 0];


[t,y] = ode45(@(t,x) robust_track(t,x,K,P), tspan,x0);

% Desired Trajectory --
theta_1d = deg2rad(180)*(1-0.03.*t.^2+0.002.*t.^3);
theta_2d = deg2rad(90)*(1-0.03.*t.^2+0.002.*t.^3);
theta_dot_1d = deg2rad(180)*(-0.06.*t+0.006.*t.^2);
theta_dot_2d = deg2rad(90)*(-0.06.*t+0.006.*t.^2);
theta_ddot_1d = deg2rad(180)*(-0.06+0.012.*t);
theta_ddot_2d = deg2rad(90)*(-0.06+0.012.*t);

% disp(y(:,:));

error = transpose(y(:,1:4) - [theta_1d, theta_2d, theta_dot_1d, theta_dot_2d]);

v = -K*error + [theta_ddot_1d';theta_ddot_2d'];

%---------------------
Tau = [];

for index = 1:length(t)
    time = t(index);
    x = y(index,:).';
    [~,tau] = robust_track(time,x,K,P);
    Tau = [Tau, tau];
end

figure;
subplot(3,3,1);
plot(t,rad2deg(y(:,1)),'LineWidth',2);
hold on;
plot(t,rad2deg(theta_1d),'r','LineWidth',2.2);
xlabel('t','FontSize',12);
ylabel('theta_1 (deg)','FontSize',12);

subplot(3,3,2);
plot(t,rad2deg(y(:,2)),'g');
hold on;
plot(t,rad2deg(theta_2d),'r','LineWidth',2.2);
xlabel('t','FontSize',12);
ylabel('theta_2 (deg)','FontSize',12);

subplot(3,3,3);
plot(t,y(:,3),'b');
hold on;
plot(t,theta_dot_1d,'r','LineWidth',2.2);
xlabel('t','FontSize',12);
ylabel('theta_dot_1','FontSize',12);

subplot(3,3,4);
plot(t,y(:,4),'r');
hold on;
plot(t,theta_dot_2d,'r','LineWidth',2.2);
xlabel('t','FontSize',12);
ylabel('theta_dot_2','FontSize',12);

subplot(3,3,5);
plot(t,Tau(1,:),'g');
xlabel('t','FontSize',12);
ylabel('t1','FontSize',12);

subplot(3,3,6);
plot(t,Tau(2,:),'b');
xlabel('t','FontSize',12);
ylabel('t2','FontSize',12);


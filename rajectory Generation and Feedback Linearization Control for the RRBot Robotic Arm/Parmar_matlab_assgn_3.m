% Nikunj Parmar
syms theta_1 theta_2 theta_dot_1 theta_dot_2 theta_ddot_1 theta_ddot_2 dl_dtheta1 dl_dtheta2 dl_dtheta1_dot dl_dtheta2_dot ddl_dtheta1_dot_dt ddl_dtheta2_dot_dt 'real';
syms r1 r2 l1 l2 m1 m2 I1 I2 t1 t2 g K P 'real';
syms eq1 eq2;
syms x1 x2 x3 x4 'real';

% K --> Kinetic Energy
% P --> Potential Energy
% t1, t2 --> toques

K = (0.5*(theta_dot_1)^2)*(m1*(r1^2) + I1) + (0.5*m2)*( (l1*theta_dot_1)^2 + (r2*(theta_dot_1+theta_dot_2))^2 + 2*l1*theta_dot_1*r2*(theta_dot_1+theta_dot_2)*cos(theta_2) ) + 0.5*I2*((theta_dot_1+theta_dot_2)^2);

P = m1*g*r1*cos(theta_1) + m2*g*(l1*cos(theta_1) + r2*cos(theta_1+theta_2));

% Lagrange Equation
% L = K - P

L = K - P;

dl_dtheta1 = jacobian(L,theta_1);
dl_dtheta2 = jacobian(L,theta_2);

dl_dtheta1_dot = jacobian(L, theta_dot_1);
dl_dtheta2_dot = jacobian(L, theta_dot_2);


ddl_dtheta1_dot_dt = jacobian(dl_dtheta1_dot, [theta_1; theta_dot_1])*[theta_dot_1; theta_ddot_1] + jacobian(dl_dtheta1_dot, [theta_2; theta_dot_2])*[theta_dot_2; theta_ddot_2];
ddl_dtheta2_dot_dt = jacobian(dl_dtheta2_dot, [theta_1; theta_dot_1])*[theta_dot_1; theta_ddot_1] + jacobian(dl_dtheta2_dot, [theta_2; theta_dot_2])*[theta_dot_2; theta_ddot_2];

eq1 = ddl_dtheta1_dot_dt - dl_dtheta1 - t1;
eq2 = ddl_dtheta2_dot_dt - dl_dtheta2 - t2;

% Solving eq1 and eq2 to get theta_ddot_1 and theta_ddot_2

sol = solve([eq1==0, eq2==0], [theta_ddot_1, theta_ddot_2]);
new_theta_ddot_1 = sol.theta_ddot_1;
new_theta_ddot_2 = sol.theta_ddot_2;

% b) Manipulator Form-------------------
eom = [eq1;eq2];

eomt_new = subs(eom,[m1,m2,l1,r1,r2,g,I1,I2,theta_dot_1,theta_dot_2,theta_ddot_1,theta_ddot_2,t1,t2],[1,1,1,0.45,0.45,9.81,0.084,0.084,0,0,0,0,0,0]);

gq = subs(eom,[m1,m2,l1,r1,r2,g,I1,I2,theta_dot_1,theta_dot_2,theta_ddot_1,theta_ddot_2,t1,t2],[1,1,1,0.45,0.45,9.81,0.084,0.084,0,0,0,0,0,0]);

cq_qDot = subs(eom,[m1,m2,l1,r1,r2,g,I1,I2,theta_ddot_1,theta_ddot_2,t1,t2],[1,1,1,0.45,0.45,9.81,0.084,0.084,0,0,0,0]) - gq;

Mq_qDDot = subs(eom,[m1,m2,l1,r1,r2,g,I1,I2,t1,t2],[1,1,1,0.45,0.45,9.81,0.084,0.084,0,0]) - cq_qDot - gq;


%% manipulator form------------------------------------------------

%Mq = [(m1*r1^2+I1+I2+(m2*(2*l1^2+4*cos(theta_2)*l1*r2+2*r2^2))/2) (I2+(m2*(2*r2^2+2*l1*cos(theta_2)*r2))/2);
%      (I2+(m2*(2*r2^2+2*l1*cos(theta_2)*r2))/2) (m2*r2^2+I2)];
%Cq = [-2*m2*I1*r2*sin(theta_2)*theta_dot_2 m2*(theta_dot_2^2)*I1*r2*sin(theta_2);
%      m2*I1*r2*theta_dot_1*sin(theta_2) 0];
%gq = [-g*m2*(r2*sin(theta_1+theta_2)+l1*sin(theta_1))-g*m1*r1*sin(theta_1);
%      -g*m2*r2*sin(theta_1+theta_2)];

%[t1;t2] = Mq * [theta_ddot_1;theta_ddot_2] + Cq*[theta_dot_1;theta_dot_2] + gq;

% b) ---------------------------------------
u = sym('u',[2,1]);
u(1) = t1;
u(2) = t2;


% state space representation of the system

A = [0 0 1 0;0 0 0 1;0 0 0 0;0 0 0 0]; 
B = [0 0;0 0;1 0;0 1];

%e) -----------------------------------------
%lambda = [-1.4-1.4i,-1.4+1.4i,-1.3+i,-1.3-i];
%lambda = [-3-1.4i,-3+1.4i,-2+1i,-2-1i];
lambda = [-0.8,-0.8,-1.8,-1.8];

Kc = place(A,B,lambda);
disp(Kc);
%f) -------------------------------------

%K = [2.6900   0.0000    2.6000   0.0000;
%    0.0000    3.9200    0.0000    2.8000];
%K =   [4.1057    1.3234    3.3382    0.3156;
%   -1.4457    4.3078   -0.4762    3.4618];
%K = [7.4276    0.2097    5.0110   -0.3988;
%   -0.1902    7.3725    0.4012    4.9890];
K = [    1.4400         0    2.6000         0;
         0    1.4400         0    2.6000];


%K = [5.44 0 2.4 0;0 1.5 0 2.5];

[t, y] = ode45(@ode_planner_2link, [0, 10], [deg2rad(200), deg2rad(125), 0, 0]);

% Desired Trajectory --
theta_1d = deg2rad(180)*(1-0.03.*t.^2+0.002.*t.^3);
theta_2d = deg2rad(90)*(1-0.03.*t.^2+0.002.*t.^3);
theta_dot_1d = deg2rad(180)*(-0.06.*t+0.006.*t.^2);
theta_dot_2d = deg2rad(90)*(-0.06.*t+0.006.*t.^2);
theta_ddot_1d = deg2rad(180)*(-0.06+0.012.*t);
theta_ddot_2d = deg2rad(90)*(-0.06+0.012.*t);

F = -K*y';

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
plot(t,F(1,:),'g');
xlabel('t','FontSize',12);
ylabel('t1','FontSize',12);

subplot(3,3,6);
plot(t,F(2,:),'b');
xlabel('t','FontSize',12);
ylabel('t2','FontSize',12);


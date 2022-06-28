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

disp(dl_dtheta2_dot);

ddl_dtheta1_dot_dt = jacobian(dl_dtheta1_dot, [theta_1; theta_dot_1])*[theta_dot_1; theta_ddot_1] + jacobian(dl_dtheta1_dot, [theta_2; theta_dot_2])*[theta_dot_2; theta_ddot_2];
ddl_dtheta2_dot_dt = jacobian(dl_dtheta2_dot, [theta_1; theta_dot_1])*[theta_dot_1; theta_ddot_1] + jacobian(dl_dtheta2_dot, [theta_2; theta_dot_2])*[theta_dot_2; theta_ddot_2];

eq1 = ddl_dtheta1_dot_dt - dl_dtheta1 - t1;
eq2 = ddl_dtheta2_dot_dt - dl_dtheta2 - t2;

% Solving eq1 and eq2 to get theta_ddot_1 and theta_ddot_2

sol = solve([eq1==0, eq2==0], [theta_ddot_1, theta_ddot_2]);
new_theta_ddot_1 = sol.theta_ddot_1;
new_theta_ddot_2 = sol.theta_ddot_2;

% a)-------------------
eom = [eq1;eq2];

eomt_new = subs(eom,[m1,m2,l1,r1,r2,g,I1,I2,theta_dot_1,theta_dot_2,theta_ddot_1,theta_ddot_2,t1,t2],[1,1,1,0.45,0.45,9.81,0.084,0.084,0,0,0,0,0,0]);

sol_eq = solve(eomt_new == 0,[theta_1,theta_2]);

eql_1 = sol_eq.theta_1;
eql_2 = sol_eq.theta_2;

%% manipulator form------------------------------------------------

Mq = [(m1*r1^2+I1+I2+(m2*(2*l1^2+4*cos(theta_2)*l1*r2+2*r2^2))/2) (I2+(m2*(2*r2^2+2*l1*cos(theta_2)*r2))/2);
      (I2+(m2*(2*r2^2+2*l1*cos(theta_2)*r2))/2) (m2*r2^2+I2)];
Cq = [-2*m2*I1*r2*sin(theta_2)*theta_dot_2 m2*(theta_dot_2^2)*I1*r2*sin(theta_2);
      m2*I1*r2*theta_dot_1*sin(theta_2) 0];
gq = [-g*m2*(r2*sin(theta_1+theta_2)+l1*sin(theta_1))-g*m1*r1*sin(theta_1);
      -g*m2*r2*sin(theta_1+theta_2)];

[t1;t2] = Mq * [theta_ddot_1;theta_ddot_2] + Cq*[theta_dot_1;theta_dot_2] + gq;

% b) ---------------------------------------
u = sym('u',[2,1]);
u(1) = t1;
u(2) = t2;

% state space representation of the system

dX(1) = theta_dot_1;
dX(2) = theta_dot_2;
dX(3) = (I2*t1 - I2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + l1*m2^2*r2^3*theta_dot_1^2*sin(theta_2) + l1*m2^2*r2^3*theta_dot_2^2*sin(theta_2) + g*l1*m2^2*r2^2*sin(theta_1) + I2*g*l1*m2*sin(theta_1) + I2*g*m1*r1*sin(theta_1) - l1*m2*r2*t2*cos(theta_2) + 2*l1*m2^2*r2^3*theta_dot_1*theta_dot_2*sin(theta_2) + l1^2*m2^2*r2^2*theta_dot_1^2*cos(theta_2)*sin(theta_2) - g*l1*m2^2*r2^2*sin(theta_1 + theta_2)*cos(theta_2) + I2*l1*m2*r2*theta_dot_1^2*sin(theta_2) + I2*l1*m2*r2*theta_dot_2^2*sin(theta_2) + g*m1*m2*r1*r2^2*sin(theta_1) + 2*I2*l1*m2*r2*theta_dot_1*theta_dot_2*sin(theta_2))/(- l1^2*m2^2*r2^2*cos(theta_2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dX(4) = -(I2*t1 - I1*t2 - I2*t2 - l1^2*m2*t2 - m1*r1^2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + l1*m2^2*r2^3*theta_dot_1^2*sin(theta_2) + l1^3*m2^2*r2*theta_dot_1^2*sin(theta_2) + l1*m2^2*r2^3*theta_dot_2^2*sin(theta_2) - g*l1^2*m2^2*r2*sin(theta_1 + theta_2) - I1*g*m2*r2*sin(theta_1 + theta_2) + g*l1*m2^2*r2^2*sin(theta_1) + I2*g*l1*m2*sin(theta_1) + I2*g*m1*r1*sin(theta_1) + l1*m2*r2*t1*cos(theta_2) - 2*l1*m2*r2*t2*cos(theta_2) + 2*l1*m2^2*r2^3*theta_dot_1*theta_dot_2*sin(theta_2) + 2*l1^2*m2^2*r2^2*theta_dot_1^2*cos(theta_2)*sin(theta_2) + l1^2*m2^2*r2^2*theta_dot_2^2*cos(theta_2)*sin(theta_2) - g*l1*m2^2*r2^2*sin(theta_1 + theta_2)*cos(theta_2) + g*l1^2*m2^2*r2*cos(theta_2)*sin(theta_1) - g*m1*m2*r1^2*r2*sin(theta_1 + theta_2) + I1*l1*m2*r2*theta_dot_1^2*sin(theta_2) + I2*l1*m2*r2*theta_dot_1^2*sin(theta_2) + I2*l1*m2*r2*theta_dot_2^2*sin(theta_2) + g*m1*m2*r1*r2^2*sin(theta_1) + 2*l1^2*m2^2*r2^2*theta_dot_1*theta_dot_2*cos(theta_2)*sin(theta_2) + l1*m1*m2*r1^2*r2*theta_dot_1^2*sin(theta_2) + 2*I2*l1*m2*r2*theta_dot_1*theta_dot_2*sin(theta_2) + g*l1*m1*m2*r1*r2*cos(theta_2)*sin(theta_1))/(- l1^2*m2^2*r2^2*cos(theta_2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);

x = [x1,x2,x3,x4];
dx = [dX(1);dX(2);dX(3);dX(4)];

dx = subs(dx,[m1,m2,l1,r1,r2,g,I1,I2,theta_dot_1,theta_dot_2,theta_1,theta_2],[1,1,1,0.45,0.45,9.81,0.084,0.084,x3,x4,x1,x2]);

A = jacobian(dx,x);
B = jacobian(dx,u);

% c) Stability check at each equillibrium point----------------------------------------------------------------------------

A1 = subs(A,[x1,x2,x3,x4],[0,0,0,0]);
A1 = double(A1);
A2 = subs(A,[x1,x2,x3,x4],[pi,0,0,0]);
A2 = double(A2);
A3 = subs(A,[x1,x2,x3,x4],[0,pi,0,0]);
A3 = double(A3);

%eigen values at equilibrium points
eig_1 = eig(A1);
eig_2 = eig(A2);
eig_3 = eig(A3);

B1 = subs(B,[x1,x2,x3,x4],[0,0,0,0]);
B1 = double(B1);
B2 = subs(B,[x1,x2,x3,x4],[pi,0,0,0]);
B2 = double(B2);
B3 = subs(B,[x1,x2,x3,x4],[0,pi,0,0]);
B3 = double(B3);

%disp(eig_1); disp(eig_2); disp(eig_3);

%d) controllibility corrosponding to the upward config-------------------------------------------
rankCO = rank(ctrb(A1,B1));
%display(rankCO);
%e) -----------------------------------------
lambda = [-1.4-1.4i,-1.4+1.4i,-1.3+i,-1.3-i];
Kc = place(A1,B1,lambda);

%disp(Kc);
%f) -------------------------------------

K = [25.4537, 7.6688, 6.4562, 2.1219; 6.4544, 5.6456, 1.9244, 0.8198];
[t, y] = ode45(@ode_planner_2link, [0, 10], [deg2rad(30), deg2rad(45), 0, 0]);

F = -K*y';

figure;
subplot(3,3,1);
plot(t,rad2deg(y(:,1)),'r');
xlabel('t','FontSize',12);
ylabel('theta_1 (deg)','FontSize',12);

subplot(3,3,2);
plot(t,rad2deg(y(:,2)),'g');
xlabel('t','FontSize',12);
ylabel('theta_2 (deg)','FontSize',12);

subplot(3,3,3);
plot(t,y(:,3),'b');
xlabel('t','FontSize',12);
ylabel('theta_dot_1','FontSize',12);

subplot(3,3,4);
plot(t,y(:,4),'r');
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


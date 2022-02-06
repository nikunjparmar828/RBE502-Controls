% Nikunj Parmar
syms theta_1 theta_2 theta_dot_1 theta_dot_2 theta_ddot_1 theta_ddot_2 dl_dtheta1 dl_dtheta2 dl_dtheta1_dot dl_dtheta2_dot ddl_dtheta1_dot_dt ddl_dtheta2_dot_dt 'real';
syms r1 r2 l1 l2 m1 m2 I1 I2 t1 t2 g K P 'real';
syms eq1 eq2;

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

[t, y] = ode45(@ode_planner_2link, [0, 10], [(10*pi)/9, (25*pi)/36, 0, 0]);
plot(t, y);
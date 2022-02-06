% Nikunj Parmar

function dX = ode_planner_2link(t,X)

m1 = 1; m2 = 1; g=9.81; l1 = 1; l2 = 1; r1 = 0.45; r2 = 0.45; 
I1 = 0.084; I2 = 0.084;

dX = zeros(4,1);
X = num2cell(X);
[theta_1, theta_2,theta_dot_1, theta_dot_2] = deal(X{:});

theta_1 = wrapTo2Pi(theta_1);
theta_2 = wrapTo2Pi(theta_2);

t1 = 0; t2 = 0;

dX(1) = theta_dot_1;
dX(2) = theta_dot_2;
dX(3) = (I2*t1 - I2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + l1*m2^2*r2^3*theta_dot_1^2*sin(theta_2) + l1*m2^2*r2^3*theta_dot_2^2*sin(theta_2) + g*l1*m2^2*r2^2*sin(theta_1) + I2*g*l1*m2*sin(theta_1) + I2*g*m1*r1*sin(theta_1) - l1*m2*r2*t2*cos(theta_2) + 2*l1*m2^2*r2^3*theta_dot_1*theta_dot_2*sin(theta_2) + l1^2*m2^2*r2^2*theta_dot_1^2*cos(theta_2)*sin(theta_2) - g*l1*m2^2*r2^2*sin(theta_1 + theta_2)*cos(theta_2) + I2*l1*m2*r2*theta_dot_1^2*sin(theta_2) + I2*l1*m2*r2*theta_dot_2^2*sin(theta_2) + g*m1*m2*r1*r2^2*sin(theta_1) + 2*I2*l1*m2*r2*theta_dot_1*theta_dot_2*sin(theta_2))/(- l1^2*m2^2*r2^2*cos(theta_2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dX(4) = -(I2*t1 - I1*t2 - I2*t2 - l1^2*m2*t2 - m1*r1^2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + l1*m2^2*r2^3*theta_dot_1^2*sin(theta_2) + l1^3*m2^2*r2*theta_dot_1^2*sin(theta_2) + l1*m2^2*r2^3*theta_dot_2^2*sin(theta_2) - g*l1^2*m2^2*r2*sin(theta_1 + theta_2) - I1*g*m2*r2*sin(theta_1 + theta_2) + g*l1*m2^2*r2^2*sin(theta_1) + I2*g*l1*m2*sin(theta_1) + I2*g*m1*r1*sin(theta_1) + l1*m2*r2*t1*cos(theta_2) - 2*l1*m2*r2*t2*cos(theta_2) + 2*l1*m2^2*r2^3*theta_dot_1*theta_dot_2*sin(theta_2) + 2*l1^2*m2^2*r2^2*theta_dot_1^2*cos(theta_2)*sin(theta_2) + l1^2*m2^2*r2^2*theta_dot_2^2*cos(theta_2)*sin(theta_2) - g*l1*m2^2*r2^2*sin(theta_1 + theta_2)*cos(theta_2) + g*l1^2*m2^2*r2*cos(theta_2)*sin(theta_1) - g*m1*m2*r1^2*r2*sin(theta_1 + theta_2) + I1*l1*m2*r2*theta_dot_1^2*sin(theta_2) + I2*l1*m2*r2*theta_dot_1^2*sin(theta_2) + I2*l1*m2*r2*theta_dot_2^2*sin(theta_2) + g*m1*m2*r1*r2^2*sin(theta_1) + 2*l1^2*m2^2*r2^2*theta_dot_1*theta_dot_2*cos(theta_2)*sin(theta_2) + l1*m1*m2*r1^2*r2*theta_dot_1^2*sin(theta_2) + 2*I2*l1*m2*r2*theta_dot_1*theta_dot_2*sin(theta_2) + g*l1*m1*m2*r1*r2*cos(theta_2)*sin(theta_1))/(- l1^2*m2^2*r2^2*cos(theta_2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);

end
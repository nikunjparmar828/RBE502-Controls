% Nikunj Parmar

function dX = ode_planner_2link(t,X)

m1 = 1; m2 = 1; g=9.81; l1 = 1; l2 = 1; r1 = 0.45; r2 = 0.45; 
I1 = 0.084; I2 = 0.084;

dX = zeros(4,1);
X = num2cell(X);
[theta_1, theta_2,theta_dot_1, theta_dot_2] = deal(X{:});

theta_1d = deg2rad(180)*(1-0.03.*t.^2+0.002.*t.^3);
theta_2d = deg2rad(90)*(1-0.03.*t.^2+0.002.*t.^3);
theta_dot_1d = deg2rad(180)*(-0.06.*t+0.006.*t.^2);
theta_dot_2d = deg2rad(90)*(-0.06.*t+0.006.*t.^2);
theta_ddot_1d = deg2rad(180)*(-0.06+0.012.*t);
theta_ddot_2d = deg2rad(90)*(-0.06+0.012.*t);

if(abs(theta_1d)>2*pi)
    theta_1d = mod(theta_1d,2*pi);
end

if(abs(theta_2d)>2*pi)
    theta_2d = mod(theta_2d,2*pi);
end


%[180-(27/5)*t^2+(9/25)*t^3;90-(2.7*t^2)+(9/50)*t^3;(-54/5)*t+(27/25)*t^2;(-5.4)*t+(27/50)*t^2];

%K = [2.6900   0.0000    2.6000   0.0000;
%    0.0000    3.9200    0.0000    2.8000];
%K =   [4.1057    1.3234    3.3382    0.3156;
%   -1.4457    4.3078   -0.4762    3.4618];
%K = [7.4276    0.2097    5.0110   -0.3988;
%   -0.1902    7.3725    0.4012    4.9890];
K = [    1.4400         0    2.6000         0;
         0    1.4400         0    2.6000];
%K = [5.44 0 2.4 0;0 1.5 0 2.5];

v = -K*([theta_1; theta_2; theta_dot_1; theta_dot_2]-[theta_1d; theta_2d; theta_dot_1d; theta_dot_2d]) + [theta_ddot_1d;theta_ddot_2d];

F = [v(1)*((9*cos(theta_2))/10+1573/1000)+v(2)*((9*cos(theta_2))/20+573/2000);(573*v(2))/2000+v(1)*((9*cos(theta_2))/20+573/2000)] + [-(theta_dot_2*((9*sin(theta_2)*(theta_dot_1+theta_dot_2))/10+(9*theta_dot_1*sin(theta_2))/10))/2;(9*theta_dot_1*sin(theta_2)*(theta_dot_1+theta_dot_2))/20-(9*theta_dot_1*theta_dot_2*sin(theta_2))/20] + [-(8829*sin(theta_1+theta_2))/2000-(28449*sin(theta_1))/2000;-(8829*sin(theta_1+theta_2))/2000];

t1 = F(1); t2 = F(2);

dX(1) = theta_dot_1;
dX(2) = theta_dot_2;
dX(3) = (I2*t1 - I2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + l1*m2^2*r2^3*theta_dot_1^2*sin(theta_2) + l1*m2^2*r2^3*theta_dot_2^2*sin(theta_2) + g*l1*m2^2*r2^2*sin(theta_1) + I2*g*l1*m2*sin(theta_1) + I2*g*m1*r1*sin(theta_1) - l1*m2*r2*t2*cos(theta_2) + 2*l1*m2^2*r2^3*theta_dot_1*theta_dot_2*sin(theta_2) + l1^2*m2^2*r2^2*theta_dot_1^2*cos(theta_2)*sin(theta_2) - g*l1*m2^2*r2^2*sin(theta_1 + theta_2)*cos(theta_2) + I2*l1*m2*r2*theta_dot_1^2*sin(theta_2) + I2*l1*m2*r2*theta_dot_2^2*sin(theta_2) + g*m1*m2*r1*r2^2*sin(theta_1) + 2*I2*l1*m2*r2*theta_dot_1*theta_dot_2*sin(theta_2))/(- l1^2*m2^2*r2^2*cos(theta_2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dX(4) = -(I2*t1 - I1*t2 - I2*t2 - l1^2*m2*t2 - m1*r1^2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + l1*m2^2*r2^3*theta_dot_1^2*sin(theta_2) + l1^3*m2^2*r2*theta_dot_1^2*sin(theta_2) + l1*m2^2*r2^3*theta_dot_2^2*sin(theta_2) - g*l1^2*m2^2*r2*sin(theta_1 + theta_2) - I1*g*m2*r2*sin(theta_1 + theta_2) + g*l1*m2^2*r2^2*sin(theta_1) + I2*g*l1*m2*sin(theta_1) + I2*g*m1*r1*sin(theta_1) + l1*m2*r2*t1*cos(theta_2) - 2*l1*m2*r2*t2*cos(theta_2) + 2*l1*m2^2*r2^3*theta_dot_1*theta_dot_2*sin(theta_2) + 2*l1^2*m2^2*r2^2*theta_dot_1^2*cos(theta_2)*sin(theta_2) + l1^2*m2^2*r2^2*theta_dot_2^2*cos(theta_2)*sin(theta_2) - g*l1*m2^2*r2^2*sin(theta_1 + theta_2)*cos(theta_2) + g*l1^2*m2^2*r2*cos(theta_2)*sin(theta_1) - g*m1*m2*r1^2*r2*sin(theta_1 + theta_2) + I1*l1*m2*r2*theta_dot_1^2*sin(theta_2) + I2*l1*m2*r2*theta_dot_1^2*sin(theta_2) + I2*l1*m2*r2*theta_dot_2^2*sin(theta_2) + g*m1*m2*r1*r2^2*sin(theta_1) + 2*l1^2*m2^2*r2^2*theta_dot_1*theta_dot_2*cos(theta_2)*sin(theta_2) + l1*m1*m2*r1^2*r2*theta_dot_1^2*sin(theta_2) + 2*I2*l1*m2*r2*theta_dot_1*theta_dot_2*sin(theta_2) + g*l1*m1*m2*r1*r2*cos(theta_2)*sin(theta_1))/(- l1^2*m2^2*r2^2*cos(theta_2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);

end
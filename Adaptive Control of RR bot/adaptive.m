% Nikunj Parmar
function dX = adaptive(t,X)
% function [dX, tau] = robust_track(t,X,K,P,Phi)

m1 = 1; m2 = 1; g=9.81; l1 = 1; l2 = 1; r1 = 0.45; r2 = 0.45; 
I1 = 0.084; I2 = 0.084;
% m1_hat = 0.75; m2_hat = 0.75;
% I1_hat = 0.063; I2_hat = 0.063;

dX = zeros(4,1);
X = num2cell(X);
[theta_1, theta_2,theta_dot_1, theta_dot_2, a1, a2, a3, a4, a5] = deal(X{:});


A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];

lambda = [-3 -3 -4 -4];
K = place(A, B, lambda);

Acl = A - B*K;
Q = eye(4).*10;
P = lyap(Acl',Q);
% P = 0;

%Trajectory generation states
theta_1d = deg2rad(180)*(1-0.03*t^2+0.002*t^3);
theta_2d = deg2rad(90)*(1-0.03*t^2+0.002*t^3);
theta_dot_1d = deg2rad(180)*(-0.06*t+0.006*t^2);
theta_dot_2d = deg2rad(90)*(-0.06*t+0.006*t^2);
theta_ddot_1d = deg2rad(180)*(-0.06+0.012*t);
theta_ddot_2d = deg2rad(90)*(-0.06+0.012*t);

if(abs(theta_1d)>2*pi)
    theta_1d = mod(theta_1d,2*pi);
end

if(abs(theta_2d)>2*pi)
    theta_2d = mod(theta_2d,2*pi);
end


M_hat= [a1+2*a2*cos(theta_2), a3+a2*cos(theta_2); a3+a2*cos(theta_2), a3];
C_hat= [-a2*sin(theta_2)*theta_dot_2, -a2*sin(theta_2)*(theta_dot_1+theta_dot_2); a2*sin(theta_2)*theta_dot_1,0];
G_hat= [-a4*g*sin(theta_1)-a5*g*sin(theta_1+theta_2); -a5*g*sin(theta_1+theta_2)];


error = [theta_1; theta_2] - [theta_1d; theta_2d];
error_dot = [theta_dot_1; theta_dot_2] - [theta_dot_1d; theta_dot_2d];


v = [theta_ddot_1d;theta_ddot_2d]- K*[error; error_dot];

U = M_hat * v + C_hat * [theta_dot_1; theta_dot_2] + G_hat;
u1 = U(1,:); u2 = U(2,:);

M = [I1+I2+m1*r1^2+m2*r2^2+m2*l1^2+2*m2*l1*r2*cos(theta_2),I2+m2*r2^2+m2*r2*l1*cos(theta_2);I2+m2*r2^2+m2*r2*l1*cos(theta_2),I2+m2*r2^2];
C = [0,-m2*l1*r2*sin(theta_2)*(2*theta_dot_1+theta_dot_2);m2*l1*r2*sin(theta_2)*theta_dot_1,0];  
G = [-m2*g*r2*sin(theta_1+theta_2)-g*l1*m2*sin(theta_1)-g*m1*r1*sin(theta_1);-g*m2*r2*sin(theta_1+theta_2)];

theta_ddots = M\(U - C*[theta_dot_1; theta_dot_2] - G);

dtt1 = theta_ddots(1); dtt2 = theta_ddots(2);

Y = [dtt1, ...
cos(theta_2)*(2*dtt1+ dtt2) - 2*sin(theta_2)*theta_dot_1*theta_dot_2- sin(theta_2)*theta_dot_2^2, ...
dtt2, ...
-sin(theta_1)*g, ...
-sin(theta_1+ theta_2)*g; ...
0, ...
sin(theta_2)*theta_dot_1^2 + cos(theta_2)*dtt1, ...
dtt1 + dtt2, ...
0, ...
-sin(theta_1+theta_2)*g];


phi = M_hat\Y;
gamma = eye(5)*0.3;
% disp('ola!')
est_alpha_dot = -gamma\(phi'*B'*P*[error; error_dot]);


dX(1) = theta_dot_1;
dX(2) = theta_dot_2;
% dX(3) = (I2*u1 - I2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + theta_dot_1^2*l1*m2^2*r2^3*sin(theta_2) + theta_dot_2^2*l1*m2^2*r2^3*sin(theta_2) + g*l1*m2^2*r2^2*sin(theta_1) + I2*g*l1*m2*sin(theta_1) + I2*g*m1*r1*sin(theta_1) - l1*m2*r2*u2*cos(theta_2) + 2*theta_dot_1*theta_dot_2*l1*m2^2*r2^3*sin(theta_2) + theta_dot_1^2*l1^2*m2^2*r2^2*cos(theta_2)*sin(theta_2) - g*l1*m2^2*r2^2*sin(theta_1 + theta_2)*cos(theta_2) + I2*theta_dot_1^2*l1*m2*r2*sin(theta_2) + I2*theta_dot_2^2*l1*m2*r2*sin(theta_2) + g*m1*m2*r1*r2^2*sin(theta_1) + 2*I2*theta_dot_1*theta_dot_2*l1*m2*r2*sin(theta_2))/(- l1^2*m2^2*r2^2*cos(theta_2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
% dX(4) = -(I2*u1 - I1*u2 - I2*u2 - l1^2*m2*u2 - m1*r1^2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + theta_dot_1^2*l1*m2^2*r2^3*sin(theta_2) + theta_dot_1^2*l1^3*m2^2*r2*sin(theta_2) + theta_dot_2^2*l1*m2^2*r2^3*sin(theta_2) - g*l1^2*m2^2*r2*sin(theta_1 + theta_2) - I1*g*m2*r2*sin(theta_1 + theta_2) + g*l1*m2^2*r2^2*sin(theta_1) + I2*g*l1*m2*sin(theta_1) + I2*g*m1*r1*sin(theta_1) + l1*m2*r2*u1*cos(theta_2) - 2*l1*m2*r2*u2*cos(theta_2) + 2*theta_dot_1*theta_dot_2*l1*m2^2*r2^3*sin(theta_2) + 2*theta_dot_1^2*l1^2*m2^2*r2^2*cos(theta_2)*sin(theta_2) + theta_dot_2^2*l1^2*m2^2*r2^2*cos(theta_2)*sin(theta_2) - g*l1*m2^2*r2^2*sin(theta_1 + theta_2)*cos(theta_2) + g*l1^2*m2^2*r2*cos(theta_2)*sin(theta_1) + I1*theta_dot_1^2*l1*m2*r2*sin(theta_2) + I2*theta_dot_1^2*l1*m2*r2*sin(theta_2) + I2*theta_dot_2^2*l1*m2*r2*sin(theta_2) - g*m1*m2*r1^2*r2*sin(theta_1 + theta_2) + g*m1*m2*r1*r2^2*sin(theta_1) + 2*theta_dot_1*theta_dot_2*l1^2*m2^2*r2^2*cos(theta_2)*sin(theta_2) + theta_dot_1^2*l1*m1*m2*r1^2*r2*sin(theta_2) + 2*I2*theta_dot_1*theta_dot_2*l1*m2*r2*sin(theta_2) + g*l1*m1*m2*r1*r2*cos(theta_2)*sin(theta_1))/(- l1^2*m2^2*r2^2*cos(theta_2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dX(3) = (I2*u1 - I2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + theta_dot_1^2*l1*m2^2*r2^3*sin(theta_2) + theta_dot_2^2*l1*m2^2*r2^3*sin(theta_2) + g*l1*m2^2*r2^2*sin(theta_1) + I2*g*l1*m2*sin(theta_1) + I2*g*m1*r1*sin(theta_1) - l1*m2*r2*u2*cos(theta_2) + 2*theta_dot_1*theta_dot_2*l1*m2^2*r2^3*sin(theta_2) + theta_dot_1^2*l1^2*m2^2*r2^2*cos(theta_2)*sin(theta_2) - g*l1*m2^2*r2^2*sin(theta_1 + theta_2)*cos(theta_2) + I2*theta_dot_1^2*l1*m2*r2*sin(theta_2) + I2*theta_dot_2^2*l1*m2*r2*sin(theta_2) + g*m1*m2*r1*r2^2*sin(theta_1) + 2*I2*theta_dot_1*theta_dot_2*l1*m2*r2*sin(theta_2))/(- l1^2*m2^2*r2^2*cos(theta_2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dX(4) = -(I2*u1 - I1*u2 - I2*u2 - l1^2*m2*u2 - m1*r1^2*u2 + m2*r2^2*u1 - m2*r2^2*u2 + theta_dot_1^2*l1*m2^2*r2^3*sin(theta_2) + theta_dot_1^2*l1^3*m2^2*r2*sin(theta_2) + theta_dot_2^2*l1*m2^2*r2^3*sin(theta_2) - g*l1^2*m2^2*r2*sin(theta_1 + theta_2) - I1*g*m2*r2*sin(theta_1 + theta_2) + g*l1*m2^2*r2^2*sin(theta_1) + I2*g*l1*m2*sin(theta_1) + I2*g*m1*r1*sin(theta_1) + l1*m2*r2*u1*cos(theta_2) - 2*l1*m2*r2*u2*cos(theta_2) + 2*theta_dot_1*theta_dot_2*l1*m2^2*r2^3*sin(theta_2) + 2*theta_dot_1^2*l1^2*m2^2*r2^2*cos(theta_2)*sin(theta_2) + theta_dot_2^2*l1^2*m2^2*r2^2*cos(theta_2)*sin(theta_2) - g*l1*m2^2*r2^2*sin(theta_1 + theta_2)*cos(theta_2) + g*l1^2*m2^2*r2*cos(theta_2)*sin(theta_1) + I1*theta_dot_1^2*l1*m2*r2*sin(theta_2) + I2*theta_dot_1^2*l1*m2*r2*sin(theta_2) + I2*theta_dot_2^2*l1*m2*r2*sin(theta_2) - g*m1*m2*r1^2*r2*sin(theta_1 + theta_2) + g*m1*m2*r1*r2^2*sin(theta_1) + 2*theta_dot_1*theta_dot_2*l1^2*m2^2*r2^2*cos(theta_2)*sin(theta_2) + theta_dot_1^2*l1*m1*m2*r1^2*r2*sin(theta_2) + 2*I2*theta_dot_1*theta_dot_2*l1*m2*r2*sin(theta_2) + g*l1*m1*m2*r1*r2*cos(theta_2)*sin(theta_1))/(- l1^2*m2^2*r2^2*cos(theta_2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dX(5) = est_alpha_dot(1);
dX(6) = est_alpha_dot(2);
dX(7) = est_alpha_dot(3);
dX(8) = est_alpha_dot(4);
dX(9) = est_alpha_dot(5);
end
% Nikunj Parmar
function [dX,tau] = robust_track(t,X,K,P)
% function [dX, tau] = robust_track(t,X,K,P,Phi)

m1 = 1; m2 = 1; g=9.81; l1 = 1; l2 = 1; r1 = 0.45; r2 = 0.45; 
I1 = 0.084; I2 = 0.084;
m1_hat = 0.75; m2_hat = 0.75;
I1_hat = 0.063; I2_hat = 0.063;

dX = zeros(4,1);
X = num2cell(X);
[theta_1, theta_2,theta_dot_1, theta_dot_2] = deal(X{:});

%Trajectory generation states
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

Rho = 0.3;
Phi = 0.075;

B = [0 0;0 0;1 0;0 1];

error= [theta_1; theta_2; theta_dot_1; theta_dot_2]-[theta_1d; theta_2d; theta_dot_1d; theta_dot_2d]; 

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
    

v = -K*(error) + [theta_ddot_1d;theta_ddot_2d]+Vr;

%F = [v(1)*((9*cos(theta_2))/10+1573/1000)+v(2)*((9*cos(theta_2))/20+573/2000);(573*v(2))/2000+v(1)*((9*cos(theta_2))/20+573/2000)] + [-(theta_dot_2*((9*sin(theta_2)*(theta_dot_1+theta_dot_2))/10+(9*theta_dot_1*sin(theta_2))/10))/2;(9*theta_dot_1*sin(theta_2)*(theta_dot_1+theta_dot_2))/20-(9*theta_dot_1*theta_dot_2*sin(theta_2))/20] + [-(8829*sin(theta_1+theta_2))/2000-(28449*sin(theta_1))/2000;-(8829*sin(theta_1+theta_2))/2000];

F = [v(1)*((27*cos(theta_2))/40 + 4719/4000) - (85347*sin(theta_1))/8000 - (26487*sin(theta_1 + theta_2))/8000 + v(2)*((27*cos(theta_2))/80 + 1719/8000) - (3*theta_dot_2*((9*sin(theta_2)*(theta_dot_1 + theta_dot_2))/10 + (9*theta_dot_1*sin(theta_2))/10))/8;
                                             (1719*v(2))/8000 - (26487*sin(theta_1 + theta_2))/8000 + v(1)*((27*cos(theta_2))/80 + 1719/8000) + (27*theta_dot_1*sin(theta_2)*(theta_dot_1 + theta_dot_2))/80 - (27*theta_dot_1*theta_dot_2*sin(theta_2))/80];

tau = F;

Mq = [(m1*r1^2+I1+I2+(m2*(2*l1^2+4*cos(theta_2)*l1*r2+2*r2^2))/2) (I2+(m2*(2*r2^2+2*l1*cos(theta_2)*r2))/2);
      (I2+(m2*(2*r2^2+2*l1*cos(theta_2)*r2))/2) (m2*r2^2+I2)];
Cq = [-2*m2*I1*r2*sin(theta_2)*theta_dot_2 m2*(theta_dot_2^2)*I1*r2*sin(theta_2);
      m2*I1*r2*theta_dot_1*sin(theta_2) 0];
gq = [-g*m2*(r2*sin(theta_1+theta_2)+l1*sin(theta_1))-g*m1*r1*sin(theta_1);
      -g*m2*r2*sin(theta_1+theta_2)];


mat = inv(Mq)*(F - Cq - gq);

theta_ddot_1 = mat(1,1);
theta_ddot_2 = mat(2,1);

dX(1) = theta_dot_1;
dX(2) = theta_dot_2;
dX(3) = theta_ddot_1;
dX(4) = theta_ddot_2;

end
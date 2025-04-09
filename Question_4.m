%% Question 4

run('Setup.m')

syms tau

% State space matrices
F = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0];

G = [0 0; 
     0 0;
     1 0;
     0 1];

H = [1 0 0 0;
     0 1 0 0];

Qk = Wk*Wk';
R = (0.3^2)*eye(2);
P_m1 = 0;

Phi = eye(4) + F*dt;
Gam = double(int(Phi*G, tau, 0, dt));

x = [0;0;0;0];
u = [ins(1,2);ins(1,3)];

x_plot = [];
y_plot = [];

% for i = 1:12
%     u = [ins((i*5-4),2);ins((i*5-4),3)];
% 
%     x_m = Phi*x + Gam*u;
%     P_m = Phi*P_m1*Phi' + Qk;
%     y_m = H*x_m;
% 
%     K = (H*P_m)'*inv(H*P_m*H'+R);
% 
%     y = gps(i);
% 
%     x_p = x_m + K*(y-y_m);
%     P_p = (eye(4)-K*H)*P_m;
% 
%     x_plot(i,1) = x_p(1);
%     x_plot(i,2) = x_p(2);
%     x_plot(i,3) = x_p(3);
%     x_plot(i,4) = x_p(4);
% 
%     x_m = x_p;
%     P_m = P_p;
% 
% end
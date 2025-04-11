%% Question 4

run('Setup.m')

syms tau

% State space matrices
F = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0];

H = [1 0 0 0;
     0 1 0 0];

Pk = zeros(4);
Qk = Wk*Wk';
R = (0.3^2)*eye(2);
Phi = eye(4) + F*dt;

x(1,1:4) = deal(0); % Initialize state vector to 0 with initial conditions.

% x = [x,y,vx,vy]

for k = 2:n
    % Measurements
    theta_meas = ins(k,2);
    phi_meas = ins(k,3);
    
    % Predicting state
    x(k,1) = x(k-1,1) + x(k-1,3)*dt;
    x(k,2) = x(k-1,2) + x(k-1,4)*dt;
    x(k,3) = x(k-1,3) + (g*tan(theta_meas)+normrnd(0, Wk(3)))*dt;
    x(k,4) = x(k-1,4) + (g*tan(phi_meas)+normrnd(0, Wk(3)))*dt;
    
    % State vector before Kalman correction
    x_m = [x(k,1); x(k,2); x(k,3); x(k,4)];
  
    % Predicting covariance
    Pk = Phi*Pk*Phi'+Qk;

    % Kalman Filter correction step
    if mod(k-1,10) == 0 && k>1

        % Extract gps data
        y = gps((k-1)/10,2:3)';

        Kk = Pk*H'*inv(H*Pk*H'+R);
        
        x_m = x_m + Kk*(y - H*x_m);
        
        Pk_p = (eye(4)-Kk*H)*Pk;
    
        x(k+1,1:4) = x_m'
    end
end

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

plot(x(:,1),x(:,2))
% Phi = [1 1 -.5; 0 1 -1; 0 0 1]
% Gam = [1 .5 -1/6; 0 1 -.5; 0 0 1]
% x0 = [100 -10 0.5]'
% u = [0 2 0]'
% Q = [1 0 0; 0 0.1 0; 0 0 0.1]
% R = 1
% H = [1 0 0]
% x1m = Phi*x0+Gam*u
% P1m = Q
% R = 1
% P = P1m
% K = (H*P)'*inv(H*P*H'+R)
% x1p = x1m + K*(91.169-x1m(1))
% P1p = (eye(3)-K*H)*P1m
% x2m = Phi*x1p+Gam*u
% P2m = Phi*P1p*Phi' + Q
% P = P2m
% K = (H*P)'*inv(H*P*H'+R)
% Delta_y2 = 81.14-x2m(1)
% x2p = x2m + K*(81.14-x2m(1))
% P2p = (eye(3)-K*H)*P2m
% x3m = Phi*x2p+Gam*u
% P3m = Phi*P2p*Phi' + Q
% P = P3m
% K = (H*P)'*inv(H*P*H'+R)
% Delta_y3 = 72.591-x3m(1)
% x3p = x3m + K*(72.591-x3m(1))
% P3p = (eye(3)-K*H)*P3m
% P4m = Phi*P3p*Phi' + Q
% x4m = Phi*x3p+Gam*u
% P = P4m
% K = (H*P)'*inv(H*P*H'+R)
% Delta_y4 = 63.834-x4m(1)
% x4p = x4m + K*(63.834-x4m(1))
% P4p = (eye(3)-K*H)*P4m
% x5m = Phi*x4p+Gam*u
% P5m = Phi*P4p*Phi' + Q
% P = P5m
% K = (H*P)'*inv(H*P*H'+R)
% Delta_y5 = 56.975-x5m(1)
% x5p = x5m + K*(56.975-x5m(1))
% P5p = (eye(3)-K*H)*P5m
% x6m = Phi*x5p+Gam*u
% P6m = Phi*P5p*Phi' + Q
% P = P6m
% K = (H*P)'*inv(H*P*H'+R)
% x6p = x6m + K*(49.011-x6m(1))
% P6p = (eye(3)-K*H)*P6m
% x7m = Phi*x6p+Gam*u
% P7m = Phi*P6p*Phi' + Q
% P = P7m
% K = (H*P)'*inv(H*P*H'+R)
% x7p = x7m + K*(42.338-x7m(1))
% P7p = (eye(3)-K*H)*P7m
% x8m = Phi*x7p+Gam*u
% P8m = Phi*P7p*Phi' + Q
% P = P8m
% K = (H*P)'*inv(H*P*H'+R)
% x8p = x8m + K*(34.770-x8m(1))
% P8p = (eye(3)-K*H)*P8m
% x9m = Phi*x8p+Gam*u
% P9m = Phi*P8p*Phi' + Q
% P = P9m
% K = (H*P)'*inv(H*P*H'+R)
% x9p = x9m + K*(30.896-x9m(1))
% P9p = (eye(3)-K*H)*P9m
% x10m = Phi*x9p+Gam*u
% P10m = Phi*P9p*Phi' + Q
% P = P10m
% K = (H*P)'*inv(H*P*H'+R)
% x10p = x10m + K*(25.726-x10m(1))
% P10p = (eye(3)-K*H)*P10m

Phi = [1 1 -0.5; 0 1 -1; 0 0 1];
Gam = [1 0.5 -1/6; 0 1 -0.5; 0 0 1];
x0 = [100; -10; 0.5];
u = [0; 2; 0];
Q = [1 0 0; 0 0.1 0; 0 0 0.1];
R = 1;
H = [1 0 0];

% Measurements (y_k)
measurements = [91.169, 81.14, 72.591, 63.834, 56.975, 49.011, 42.338, 34.770, 30.896, 25.726];

% Initialize state and covariance
x = Phi * x0 + Gam * u;
P = Q;

% Kalman filter loop
for k = 1:10
    % Prediction step
    if k > 1
        x = Phi * x + Gam * u;
        P = Phi * P * Phi' + Q;
    end
    
    % Store prediction (x_k^-)
    x_pred = x;
    
    % Calculate innovation (delta)
    delta = measurements(k) - H * x_pred;
    
    % Update step
    K = (H * P)' / (H * P * H' + R);
    x = x + K * delta;
    P = (eye(3) - K * H) * P;
    
    % Display results for each step
    fprintf('Step %d:\n', k);
    fprintf('Predicted state: [%.3f, %.3f, %.3f]\n', x_pred(1), x_pred(2), x_pred(3));
    fprintf('Innovation (delta): %.3f\n', delta);
    fprintf('Corrected state: [%.3f, %.3f, %.3f]\n\n', x(1), x(2), x(3));
end

% Final results
disp('Final corrected state:');
disp(x);
disp('Final covariance matrix:');
disp(P);
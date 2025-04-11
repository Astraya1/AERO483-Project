%% Question 4

%% Kalman Filter Approximation
run('Setup.m')

syms tau

% State space matrices
F = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0];

H = [1 0 0 0;
     0 1 0 0];

Pk = zeros(4,4); % Covariance matrix
Qk = Wk*Wk'; % Noise covariance
R = (0.3^2)*eye(2); % Measurement noise covariance
Phi = eye(4) + F*dt; % State matrix

x(1,1:4) = deal(0); % Initialize state vector to 0 with initial conditions.

for k = 1:n
    % Measurements
    theta_meas = ins(k,2);
    phi_meas = ins(k,3);
    
    % Update the next state vector
    x(1) = x(1) + x(3)*dt;
    x(2) = x(2) + x(4)*dt;
    x(3) = x(3) + g*tan(theta_meas)*dt;
    x(4) = x(4) + g*tan(phi_meas)*dt;

    x = x';
  
    % Update covariance matrix
    Pk = Phi*Pk*Phi'+Qk;

    % Kalman Filter correction step
    if mod(k-1,10) == 0 && k>1

        % Extract gps data
        y = gps((k-1)/10,2:3)';
        
        % Calculate Kalman gain
        Kk = Pk*H'*inv(H*Pk*H'+R);
        
        % Update state vector with Kalman filter
        x = x + Kk*(y - H*x);
        
        % Update covariance matrix
        Pk = (eye(4)-Kk*H)*Pk;
    
    end

    x_kalman(:,k) = x';
end

%% Copied code from Question 2

% Initialize variable arrays
[x, y, vx, vy] = deal(zeros(n, 1));

% Apply initial conditions
[x(1), y(1), vx(1), vy(1)] = deal(0);

% Simulate using Euler's forward method
for k = 1:n

    % Calculate acceleration terms
    ax = g * tan(theta(k));
    ay = g * tan(phi(k));
    
    % Calculate velocity deltas
    dvx = ax*dt;
    dvy = ay*dt;

    % Calculate velocity terms
    vx(k+1) = vx(k) + dvx;
    vy(k+1) = vy(k) + dvy;

    % Calculate position deltas
    dx = vx(k)*dt + 0.5*ax*dt^2;
    dy = vy(k)*dt + 0.5*ay*dt^2;

    % Calculate position terms
    x(k+1) = x(k) + dx;
    y(k+1) = y(k) + dy;
end

%% Plots

figure;
hold on;
plot(x, y, 'b');
plot(x_gps, y_gps, 'r');
plot(x_kalman(1,:),x_kalman(2,:), 'm', 'LineWidth',1.5)
plot(x_gps, y_gps, 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);

title("Kalman Filter Estimation of Simulated Quadcopter Dynamics");
xlabel('x-Position (m)');
ylabel('y-Position (m)');
legend('INS Mechanization', 'GPS Measurements', 'Kalman Filter Estimation');

grid on;
axis equal;

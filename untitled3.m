

%% Load Data
ins_data = load('ins.txt');  % Format: [time, theta, phi]
gps_data = load('gps.txt');  % Format: [time, x, y]

%% Initialize Parameters
g = 9.8;                    % Gravity (m/s^2)
dt = 0.05;                  % IMU time step (s)
sigma_GPS = 0.3;            % GPS noise standard deviation (m)

% Initial State [x; y; vx; vy]
x_hat = [0; 0; 0; 0];       % Start at origin, zero velocity

% Initial Covariance Matrix
P = zeros(4,4);             % Perfect initial knowledge (per project)

% Process Noise Covariance (Q)
w_k = [0; 0; 0.03; 0.03];   % Wind noise terms
Q = w_k * w_k';             % Q = w_k * w_k^T

% Measurement Noise Covariance (R)
R = sigma_GPS^2 * eye(2);   % R = σ² * I (2x2)

% Measurement Matrix (H)
H = [1 0 0 0;               % Maps state to GPS [x; y]
     0 1 0 0];

%% Preallocate Storage
N = length(ins_data);       % Number of IMU steps
x_est = zeros(4, N);       % Estimated state history
P_est = zeros(4,4,N);      % Covariance history
gps_update_time = 0.5;      % GPS update interval (s)
next_gps_time = 0.5;          % Time of next GPS update

%% EKF Main Loop
for k = 1:N
    theta = ins_data(k, 2);
    phi = ins_data(k, 3);
    
    % --- Prediction Step (Always) ---
    % Nonlinear state update
    x_hat(1) = x_hat(1) + x_hat(3)*dt;              % x position
    x_hat(2) = x_hat(2) + x_hat(4)*dt;              % y position
    x_hat(3) = x_hat(3) + g*tan(theta)*dt;          % vx velocity
    x_hat(4) = x_hat(4) + g*tan(phi)*dt;            % vy velocity
    
    % Linearized state transition (Jacobian)
    Phi = [1 0 dt  0;
           0 1  0 dt;
           0 0  1  0;
           0 0  0  1];
    
    % Predict covariance
    P = Phi * P * Phi' + Q;
    
    % --- Correction Step (Only at GPS times) ---
    if mod(k-1,10) == 0 && k>1% Account for floating-point errors
        % Find closest GPS measurement to current_time
       
        z_GPS = gps_data((k-1)/10,2:3)';

        % Kalman Gain
        K = P * H' / (H * P * H' + R);
        
        % State update
        x_hat = x_hat + K * (z_GPS - H * x_hat);
        
        % Covariance update
        P = (eye(4) - K * H) * P;
        
    end
    
    % Store results
    x_est(:,k) = x_hat;
    P_est(:,:,k) = P;
end

%% Plot Results
figure;
hold on; grid on;

% Plot GPS measurements (every 0.5s)
scatter(gps_data(:,2), gps_data(:,3), 'r', 'filled', 'DisplayName', 'GPS Data');

% Plot EKF estimate
plot(x_est(1,:), x_est(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'EKF Estimate');

% Plot INS-only trajectory (for comparison)
x_INS = [0; 0; 0; 0];
x_INS_history = zeros(4,N);
for k = 1:N
    theta = ins_data(k,2);
    phi = ins_data(k,3);
    x_INS(1) = x_INS(1) + x_INS(3)*dt;
    x_INS(2) = x_INS(2) + x_INS(4)*dt;
    x_INS(3) = x_INS(3) + g*tan(theta)*dt;
    x_INS(4) = x_INS(4) + g*tan(phi)*dt;
    x_INS_history(:,k) = x_INS;
end
plot(x_INS_history(1,:), x_INS_history(2,:), 'g--', 'DisplayName', 'INS Only');

title('Trajectory Estimation (GPS Updates Every 0.5s)');
xlabel('x (m)'); ylabel('y (m)');
legend('Location', 'best');
axis equal;
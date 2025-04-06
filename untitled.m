% Falcon Heavy Booster Landing Kalman Filter
clear all; close all; clc;

% System parameters
T = 1.0;            % sampling time (s)
g = 10;             % gravity (m/s^2)
a_measured = 12;    % accelerometer reading (m/s^2)
bias = 0.5;         % accelerometer bias (m/s^2)
a_true = a_measured - bias - g; % net acceleration (m/s^2)

% System matrices
Phi = [1 T; 0 1];   % state transition matrix
Gamma = [0.5*T^2; T]; % control input matrix
H = [1 0];          % measurement matrix

% Noise covariances
Q = [1 0; 0 0.1];   % process noise covariance
R = 1;              % measurement noise variance (m^2)

% Initial conditions
x_est = [100; -10];  % initial state [height; velocity]
P_est = zeros(2);    % initial covariance (perfect certainty)

% Measurement data
measurements = [91.169; 81.140; 72.591; 63.834; 56.975];

% Storage for results
estimated_states = zeros(2, length(measurements));
estimated_cov = zeros(2, 2, length(measurements));

% Kalman filter loop
for k = 1:length(measurements)
    %% Prediction Step
    % State prediction
    x_pred = Phi * x_est + Gamma * a_true;
    
    % Covariance prediction
    P_pred = Phi * P_est * Phi' + Q;
    
    %% Update Step
    % Kalman gain calculation
    K = P_pred * H' / (H * P_pred * H' + R);
    
    % Measurement update
    z = measurements(k);
    x_est = x_pred + K * (z - H * x_pred);
    
    % Covariance update
    P_est = (eye(2) - K * H) * P_pred;
    
    % Store results
    estimated_states(:,k) = x_est;
    estimated_cov(:,:,k) = P_est;
    

    
    % Display results
    fprintf('Time step %d:\n', k);
    fprintf('  Estimated height: %.3f m\n', x_est(1));
    fprintf('  Estimated velocity: %.3f m/s\n', x_est(2));
    residual = z - H * x_pred;
    fprintf('  Residual: %.3f m\n', residual);
    
    % Calculate normalized residual
    S = H * P_pred * H' + R;
    normalized_residual = residual / sqrt(S);
    fprintf('  Normalized residual: %.3f\n', normalized_residual)
    disp(P_est);
end

%% Plotting results
figure;
subplot(2,1,1);
plot(0:length(measurements), [100; estimated_states(1,:)'], 'b-o');
hold on;
plot(1:length(measurements), measurements, 'rx', 'MarkerSize', 10);
xlabel('Time step');
ylabel('Height (m)');
legend('Estimated height', 'Measured height');
grid on;

subplot(2,1,2);
plot(0:length(measurements), [-10; estimated_states(2,:)'], 'b-o');
xlabel('Time step');
ylabel('Velocity (m/s)');
legend('Estimated velocity');
grid on;
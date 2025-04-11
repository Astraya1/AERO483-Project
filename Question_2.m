%% Question 2 (NO NOISE)

run('Setup.m')

%% Euler's Forward Method Mechanization

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
plot(x_gps, y_gps, 'rx', 'MarkerSize', 8, 'LineWidth', 1.5);

title("Simulated System Dynamics of Quadcopter Without Noise");
xlabel('x-Position (m)');
ylabel('y-Position (m)');
legend('INS Measurement', 'GPS Measurement');

grid on;
axis equal;
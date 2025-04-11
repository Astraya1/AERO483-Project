%% Question 3 (WITH NOISE)

run('Setup.m')

%% INS Mechanization With Noise

figure;
hold on;

% Need to simulate to do 10 runs
for i = 1:10

    % Initialize arrays
    [x_n, y_n, vx_n, vy_n] = deal(zeros(n, 1));
    
    % Apply initial conditions
    [x_n(1), y_n(1), vx_n(1), vy_n(1)] = deal(0);
    
    %Euler's forward method, but with noise added to acceleration
    for k = 1:n

        % Calculate acceleration terms
        ax_n = g*tan(theta(k)) + normrnd(0, Wk(3));
        ay_n = g*tan(phi(k)) + normrnd(0, Wk(3));
        
        % Calculate velocity deltas
        dvx_n = ax_n*dt;
        dvy_n = ay_n*dt;
        
        % Calculate velocity terms
        vx_n(k+1) = vx_n(k) + dvx_n;
        vy_n(k+1) = vy_n(k) + dvy_n;
    
        % Calculate position deltas
        dx_n = vx_n(k)*dt + 0.5*ax_n*dt^2;
        dy_n = vy_n(k)*dt + 0.5*ay_n*dt^2;
        
        % Calculate position terms
        x_n(k+1) = x_n(k) + dx_n;
        y_n(k+1) = y_n(k) + dy_n;
    end

    plot(x_n, y_n);
end

%% Format plot
title("Simulated System Dynamics of Quadcopter With White Noise");
xlabel('x-Position (m)');
ylabel('y-Position (m)');

grid on;
axis equal;
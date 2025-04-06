%% Question 1

run('Setup.m')

% Plot pitch angle
figure;
hold on;
plot(time_ins, theta, 'b');

% Plot roll angle
plot(time_ins, phi, 'r');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Pitch Angle \theta(t)', 'Roll Angle \phi(t)');
grid on;
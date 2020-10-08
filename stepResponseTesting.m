%% Step Response Testing
% Compare the velocity step response of the motor controller to a transfer
% function and develop a PID controller to implement motor control

%% initialize variables
motorData = [];

%%
K = 6.85/2.5;
sigma = 10;
%% Plot Motor Velocity and Simulation Velocity

% Plot Motor Data
time = motorData(:,1); 
velocity = motorData(:,2); 
figure(1); plot(time, velocity); 
xlabel("time [ms]");
ylabel("angular velocity [rads/sec]");

% Run simulation and plot results against motor data
out = sim('section4_6');

figure(2); 
plot(out.simout1);
hold on;
plot(time, velocity);
hold off;



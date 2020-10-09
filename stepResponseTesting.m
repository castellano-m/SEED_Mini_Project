%% Step Response Testing
% Compare the experimental step response of a motor's angular velocity to
% the simulated step response of an open loop system modeling the motor.

%% Create empty array for motor data
% Data was taken from the Arduino serial monitor. The motor was supplied a
% 50% PWM signal at time = 1 second. The 50% PWM signal meant an analog 4V
% was supplied to the motor from the battery. Data was copied into Excel,
% where it could easily be copied into MATLAB and written into an array.

motorData = [1.00,0.00;   1.05,2.08;  1.10,4.64;  1.15,5.78;  1.20,6.25;
            1.25,6.57;  1.30,6.76;  1.35,6.84;  1.40,6.88;  1.45,6.80;
            1.50,6.84;  1.55,6.80;  1.60,6.88;  1.65,6.84;  1.70,6.88;
            1.75,6.80;  1.80,6.88;  1.85,6.84;  1.90,6.88;  1.95,6.88;
            2.00,6.88;  2.05,6.73;  2.10,6.84;  2.15,6.88;  2.20,6.88;
            2.25,6.88;  2.30,6.88;  2.35,6.76;  2.40,6.84;  2.45,6.88;
            2.50,6.88;  2.55,6.92;  2.60,6.80;  2.65,6.73;  2.70,6.92;
            2.75,6.88];

%% Initialize motor model constants
% K and sigma were calculated following the first graph of Section 4.1 in
% the EENG 307 Lecture 15 handout. K was set equal to final velocity in
% rad/s and 1/sigma was set to the time where velocity equaled 0.64*K. Due
% to the error inherent in the measurement method we were required to use,
% the final velocity of the motor data was averaged, resulting in an
% angular velocity of 6.85 rads/s. The inverse of sigma was set equal to
% the time in seconds where velocity equaled 64% of its final value.
%
% 4V were supplied to the motor as the voltage step in the experiment,
% meaning in the simulation the step input signal needs to go through a
% gain of 4 to properly mimic the experiment. To offset this immediate
% input gain, the K value in the transfer function must be divided by 4.

K = 6.85/4;
sigma = 10;

%% Plot Motor Velocity and Simulation Velocity
% Plot Motor Data
time = motorData(:,1); 
velocity = motorData(:,2); 
figure(1); plot(time, velocity); 
xlabel("time [ms]");
ylabel("angular velocity [rads/s]");

% Run simulation and plot results of simulated step response against
% experimental motor data
out = sim('section4_6');

figure(2); 
plot(out.simout1);
xlabel("time [s]");   ylabel("angular velocity [rads/s]");
hold on;
plot(time, velocity);
hold off;



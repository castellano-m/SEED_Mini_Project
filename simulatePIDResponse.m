%% Section 4.7
% Run the simulation in section4_7.slx and plot results
%
% To design the control system implemented on an Arduino, the PID tuner app
% provided by Simulink was used. The controller implemented in the
% demonstration was a PID controller. The PID tuner gave gain values for
% proportional, integral, and derivative gain as follows: 
%   K_p = 1.25 [V/rad]
%   K_i = 0.84 [V/rad*s]
%   K_d = 0.03 [V/rad/s]
% When used to control the real motor, these gain values did not produce
% the best motor response. The largest issue was the derivative gain needed
% to be significantly increased so the wheel could reach the correct
% position quickly. The implementation used to demo for the Mini Project
% Demonstration used a derivative gain around ten times larger than the
% simulated value. This introduced significant overshoot. Moving forward
% into Demonstration 1, the controller used to drive the motors will be
% changed to a PI controller instead of a PID controller, tuned to minimize
% overshoot while keeping a short rise time. Furthermore, during the
% documentation phase of the Mini Assignment, it was realized both
% simulations used an incorrect K value. Before the documentation phase,
% the K value was the average final angular velocity divided by 2.5, as a
% 50% duty cycle PWM signal results in a 2.5V analog signal from the
% digital output pin on the Arduino. The motor, however, is not driven by
% this voltage. The motor shield uses the PWM signal supplied by the
% Arduino to create a PWM wave of the external battery voltage, which in
% this case was a 50% duty cycle from an 8V source, leading to a 4V analog
% signal.

%% Initialize Variables for Motor Transfer Function
K = 6.85/4;
sigma = 10;

%% Run simulation and plot results
% The input position to the system was 3.14 radians.
out = sim('section4_7');
figure(1);
plot(out.simout);
xlabel('time [s]'); ylabel('position [rads]')
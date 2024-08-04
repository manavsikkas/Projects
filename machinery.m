% Parameters
m1 = 75; % Mass in kg
k = 6662.25; % Spring constant in N/m
c = 353.5; % Damping coefficient in Ns/m

% Define the system transfer function
numerator = [c k];
denominator = [m1 c k];
sys = tf(numerator, denominator);
% Tuned PID parameters (Provided values)
Kp_tuned = 1.9052;      % Tuned Proportional gain
Ki_tuned = 10.5114;     % Tuned Integral gain
Kd_tuned = 0;           % Tuned Derivative gain

% Create tuned PID controller
pid_tuned = pid(Kp_tuned, Ki_tuned, Kd_tuned);
open_loop_sys_cont=pid_tuned*sys;

% Closed-loop system with tuned PID controller (continuous)
closed_loop_sys_cont = feedback(pid_tuned * sys, 1);

% Calculate the closed-loop bandwidth
bw_cont = bandwidth(closed_loop_sys_cont);

% Convert bandwidth to Hz
bw_hz_cont = bw_cont / (2 * pi);

% Calculate the period corresponding to the bandwidth
period_bw = 1 / bw_hz_cont;

% Calculate the sampling time as 1/20th of the period
sampling_time = period_bw / 20;

% Display the bandwidth and sampling time
disp(['Closed-Loop Bandwidth (Continuous): ', num2str(bw_hz_cont), ' Hz']);
disp(['Period Corresponding to Bandwidth: ', num2str(period_bw), ' seconds']);
disp(['Sampling Time (1/20th of Period): ', num2str(sampling_time), ' seconds']);

% Discretize the closed-loop system using Zero-Order Hold (ZOH) approximation
closed_loop_sys_d = c2d(closed_loop_sys_cont, sampling_time, 'zoh');

% Time vector for simulation
t_disc = 0:sampling_time:10; % Discrete time vector

% Simulate the response without PID control (open-loop response)
[y_open, t_cont] = step(sys, t_disc);

% Simulate the response with tuned PID gains (discrete)
[y_tuned_d, t_disc] = step(closed_loop_sys_d, t_disc);

% Plot the results
figure;

subplot(2,1,1);
plot(t_cont, y_open, 'b', 'LineWidth', 1.5);
title('Output Displacement x(t) without PID Control (Open-Loop Response)');
xlabel('Time (s)');
ylabel('Displacement (m)');
grid on;

subplot(2,1,2);
stairs(t_disc, y_tuned_d, 'g', 'LineWidth', 1.5); % Use stairs for discrete data
title('Output Displacement x(t) with Tuned PID Control (Discrete)');
xlabel('Time (s)');
ylabel('Displacement (m)');
grid on;
closed_loop_sys_d
% Check the stability of the closed-loop system
poles_cont = pole(closed_loop_sys_cont);
poles_disc = pole(closed_loop_sys_d);
disp('Poles of the closed-loop system with tuned PID gains (Continuous):');
disp(poles_cont);
disp('Poles of the closed-loop system with tuned PID gains (Discrete):');
disp(poles_disc);

% Check if all poles are inside the unit circle for the discrete system
if all(abs(poles_disc) < 1)
    disp('The discrete closed-loop system is stable.');
else
    disp('The discrete closed-loop system is unstable.');
end

% Calculate the settling time and overshoot for the discrete system
step_info_disc = stepinfo(closed_loop_sys_d);
settling_time_disc = step_info_disc.SettlingTime;
overshoot_disc = step_info_disc.Overshoot;

disp(['Settling Time (Discrete): ', num2str(settling_time_disc), ' seconds']);
disp(['Overshoot (Discrete): ', num2str(overshoot_disc), ' %']);
% Plot the Bode plot for phase delay analysis
figure;
bode(open_loop_sys_cont);
title('Bode Plot of Open loop tuned System');
grid on;
% Calculate the phase margin, gain margin, and crossover frequencies
[gm, pm, wgm, wpm] = margin(open_loop_sys_cont);

% Display the phase margin and gain crossover frequency
disp(['Phase Margin: ', num2str(pm), ' degrees']);
disp(['Phase Crossover Frequency: ', num2str(wgm), ' rad/s']);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   X
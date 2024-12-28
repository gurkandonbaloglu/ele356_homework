

%% Gürkan DONBALOĞLU - 2220357131
% PROPORTIONAL CONTROLLER 
clc;
clear;
close all;

% System parameters
settling_time = 10; % seconds
steady_state_rpm = 230.41; % rpm
tau = settling_time / 5; % Time constant
K = steady_state_rpm; % Gain

% Continuous-time transfer function
num = [K];
den = [tau, 1];
G = tf(num, den);

% Sampling time
Ts = 0.001; % seconds

% Plot root locus
figure;
rlocus(G);
title('Root Locus of G(s)');
grid on;

% Square wave parameters
mean_rpm = 600; % Mean RPM
peak_to_peak = 800; % Peak-to-peak amplitude
period = 4; % seconds
duration = 10; % seconds

% Time vector and square wave
t = 0:Ts:duration;
%square_wave = mean_rpm + (peak_to_peak / 2) * (mod(t, period) < (period / 2)) * 2 - (peak_to_peak / 2);
square_wave = mean_rpm + (peak_to_peak / 2) * square(2 * pi * (1 / period) * t); 
% Discretize transfer function
Gd = c2d(G, Ts, 'zoh');
[A, B, C, D] = ssdata(Gd); % Extract state-space matrices

% Initialize variables for simulation
y = zeros(1, length(t)); % Preallocate output signal array
x = zeros(size(A, 1), 1); % Initial state, compatible with state-space dimensions
Kp = 0.05; % Proportional gain

% Simulation loop
for i = 1:length(t)
    % Calculate error
    if i == 1
        prev_y = 0; % For the first iteration, use 0 as previous output
    else
        prev_y = y(i-1); % Use the previous output for subsequent iterations
    end
    error = square_wave(i) - prev_y;
    
    % Calculate control signal
    u = Kp * error;
    
    % Update state using discrete-time state-space equations
    x = A * x + B * u;
    
    % Update output
    y(i) = C * x + D * u;
end

% Plot reference input and output
figure;
plot(t, square_wave, 'r', 'LineWidth', 1.5, 'DisplayName', 'Reference Signal');
hold on;
plot(t, y, 'b', 'LineWidth', 1.5, 'DisplayName', 'Output RPM');
grid on;
legend;
title('Output and Reference Signal for Proportional Controller');
xlabel('Time (s)');
ylabel('Amplitude');

%%
% Proportional-Integral Controller

Kp = 1; 
Ki = 1;

% Controller transfer function
C = tf([Kp, Ki], [1, 0]);
% Combined system
CG = series(C, G);

% Discretize PI controller and motor transfer function
Cd = c2d(C, Ts, 'zoh');
Gd = c2d(G, Ts, 'zoh');

% Extract state-space matrices for discrete-time PI controller
[Ac, Bc, Cc, Dc] = ssdata(Cd);

% Extract state-space matrices for discrete-time motor
[Ag, Bg, Cg, Dg] = ssdata(Gd);

% Initialize variables for simulation
y = zeros(1, length(t)); % Output signal
cy = 0; % Controller output
xg = zeros(size(Ag, 1), 1); % Motor state
xc = zeros(size(Ac, 1), 1); % Controller state

% Simulation loop
for i = 1:length(t)
    % Calculate error
    if i == 1
        prev_y = 0; % For the first iteration, use 0 as previous output
    else
        prev_y = y(i-1); % Use the previous output for subsequent iterations
    end
    error = square_wave(i) - prev_y;

    % Update PI controller state
    xc = Ac * xc + Bc * error;
    cy = Cc * xc + Dc * error;

    % Update motor state
    xg = Ag * xg + Bg * cy;
    y(i) = Cg * xg + Dg * cy;
end

% Plot root locus of C(s)*G(s)
figure;
rlocus(CG);
title('Root Locus of PI Controlled System');
grid on;

% Plot step response of the closed-loop system
closed_loop = feedback(CG, 1);
figure;
step(closed_loop);
title('Step Response of Closed-Loop System with PI Controller');
grid on;

% Plot reference input and output
figure;
plot(t, square_wave, 'r', 'LineWidth', 1.5, 'DisplayName', 'Reference Signal');
hold on;
plot(t, y, 'b', 'LineWidth', 1.5, 'DisplayName', 'Output RPM');
grid on;
legend;
title('Output and Reference Signal for PI Controller');
xlabel('Time (s)');
ylabel('Amplitude');


%% Q1
% Plot root locus
figure;
rlocus(G);
title('Root Locus of G(s)');
grid on;
%% Q2
% Step response
figure;
step(G);
title('Step Response of G(s)');
grid on;

%% Q3
% Square wave parameters
Ts = 0.001; % Sampling time
mean_rpm = 600; % Mean RPM
peak_to_peak = 800; % Peak-to-peak amplitude
period = 4; % seconds
duration = 10; % seconds
t = 0:Ts:duration;
square_wave = mean_rpm + (peak_to_peak / 2) * square(2 * pi * (1 / period) * t);

% Proportional gain
Kp = 0.05;

% Discretize G(s)
Gd = c2d(G, Ts, 'zoh');
[A, B, C, D] = ssdata(Gd);

% Initialize variables
y = zeros(1, length(t)); % Output signal
u = zeros(1, length(t)); % Control signal
x = zeros(size(A, 1), 1); % State vector

% Simulation loop
for i = 1:length(t)
    % Error signal
    if i == 1
        error = square_wave(i);
    else
        error = square_wave(i) - y(i-1);
    end
    
    % Control signal
    u(i) = Kp * error;
    
    % State-space update
    x = A * x + B * u(i);
    y(i) = C * x + D * u(i);
end

% Plot results
figure;
plot(t, square_wave, 'r', 'DisplayName', 'Reference Signal');
hold on;
plot(t, y, 'b', 'DisplayName', 'Output RPM');
plot(t, u, 'g', 'DisplayName', 'Control Signal');
grid on;
legend;
title('System Response for K_p = 0.05');
xlabel('Time (s)');
ylabel('Amplitude');

%% Q4
gain_values = [0.05, 0.1, 0.5, 1];

for Kp = gain_values
    % Reset output and state
    y = zeros(1, length(t));
    x = zeros(size(A, 1), 1);
    
    % Simulation loop
    for i = 1:length(t)
        if i == 1
            error = square_wave(i);
        else
            error = square_wave(i) - y(i-1);
        end
        u = Kp * error; % Control signal
        x = A * x + B * u;
        y(i) = C * x + D * u;
    end
    
    % Plot results
    figure;
    plot(t, square_wave, 'r', 'DisplayName', 'Reference Signal');
    hold on;
    plot(t, y, 'b', 'DisplayName', ['Output RPM (Kp = ', num2str(Kp), ')']);
    grid on;
    legend;
    title(['System Response for Kp = ', num2str(Kp)]);
    xlabel('Time (s)');
    ylabel('Amplitude');
end

%% PI Q1
clc;
clear;
close all;

% System parameters
settling_time = 10; % seconds
steady_state_rpm = 230.41; % rpm
tau = settling_time / 5; % Time constant
K = steady_state_rpm;

% Motor transfer function
num = [K];
den = [tau, 1];
G = tf(num, den);

% PI Controller transfer function
Kp = 1;
Ki = 1;
C = tf([Kp, Ki], [1, 0]);

% Combined system
CG = series(C, G);

% Root locus plot
figure;
rlocus(CG);
title('Root Locus of PI Controlled System');
grid on;
%% PI Q2
% Closed-loop transfer function
closed_loop = feedback(CG, 1);

% Step response
figure;
step(closed_loop);
title('Step Response of Closed-Loop System with PI Controller');
grid on;
%% PI Q3
% Sampling time
Ts = 0.001;

% Square wave parameters
mean_rpm = 600; % Mean RPM
peak_to_peak = 800; % Peak-to-peak amplitude
period = 4; % seconds
duration = 10; % seconds
t = 0:Ts:duration;
square_wave = mean_rpm + (peak_to_peak / 2) * square(2 * pi * (1 / period) * t);

% Discretize PI controller and motor
Cd = c2d(C, Ts, 'zoh');
Gd = c2d(G, Ts, 'zoh');

% Extract state-space matrices
[Ac, Bc, Cc, Dc] = ssdata(Cd); % Controller
[Ag, Bg, Cg, Dg] = ssdata(Gd); % Motor

% Initialize variables for simulation
y = zeros(1, length(t)); % Output signal
u = zeros(1, length(t)); % Control signal
xc = zeros(size(Ac, 1), 1); % Controller state
xg = zeros(size(Ag, 1), 1); % Motor state

% Simulation loop
for i = 1:length(t)
    % Calculate error
    if i == 1
        error = square_wave(i);
    else
        error = square_wave(i) - y(i-1);
    end
    
    % PI Controller state update
    xc = Ac * xc + Bc * error;
    u(i) = Cc * xc + Dc * error; % Control signal
    
    % Motor state update
    xg = Ag * xg + Bg * u(i);
    y(i) = Cg * xg + Dg * u(i); % Output signal
end

% Plot reference input and output
figure;
plot(t, square_wave, 'r', 'DisplayName', 'Reference Signal');
hold on;
plot(t, y, 'b', 'DisplayName', 'Output RPM');
plot(t, u, 'g', 'DisplayName', 'Control Signal');
grid on;
legend;
title('System Response for PI Controller (Kp = Ki = 1)');
xlabel('Time (s)');
ylabel('Amplitude');

%% PI Q4
Ki_values = [0.1, 0.5, 1, 2]; % Example values of Ki
for Ki = Ki_values
    % Update PI controller
    C = tf([Kp, Ki], [1, 0]);
    Cd = c2d(C, Ts, 'zoh');
    [Ac, Bc, Cc, Dc] = ssdata(Cd);
    
    % Reset variables
    y = zeros(1, length(t));
    u = zeros(1, length(t));
    xc = zeros(size(Ac, 1), 1);
    xg = zeros(size(Ag, 1), 1);
    
    % Simulation loop (same as above)
    for i = 1:length(t)
        if i == 1
            error = square_wave(i);
        else
            error = square_wave(i) - y(i-1);
        end
        xc = Ac * xc + Bc * error;
        u(i) = Cc * xc + Dc * error;
        xg = Ag * xg + Bg * u(i);
        y(i) = Cg * xg + Dg * u(i);
    end
    
    % Plot results
    figure;
    plot(t, square_wave, 'r', 'DisplayName', 'Reference Signal');
    hold on;
    plot(t, y, 'b', 'DisplayName', ['Output RPM (Ki = ', num2str(Ki), ')']);
    grid on;
    legend;
    title(['System Response for Ki = ', num2str(Ki)]);
    xlabel('Time (s)');
    ylabel('Amplitude');
end

%% For Both Systems

clc;
clear;
close all;

% System parameters
settling_time = 10; % seconds
steady_state_rpm = 230.41; % rpm
tau = settling_time / 5; % Time constant
K = steady_state_rpm;

% Motor transfer function
num = [K];
den = [tau, 1];
G = tf(num, den);

% Sampling time
Ts = 0.001;

% Square wave parameters
mean_rpm = 600; % Mean RPM
peak_to_peak = 800; % Peak-to-peak amplitude
period = 4; % seconds
duration = 10; % seconds
t = 0:Ts:duration;
square_wave = mean_rpm + (peak_to_peak / 2) * square(2 * pi * (1 / period) * t);

% ---- Proportional Controller (Kp = 1) ----
Kp = 1;

% Discretize motor transfer function
Gd = c2d(G, Ts, 'zoh');
[A, B, C, D] = ssdata(Gd);

% Initialize variables
y_p = zeros(1, length(t)); % Output signal
x = zeros(size(A, 1), 1); % State vector

% Simulation loop
for i = 1:length(t)
    if i == 1
        error = square_wave(i);
    else
        error = square_wave(i) - y_p(i-1);
    end
    u = Kp * error; % Control signal
    x = A * x + B * u; % State update
    y_p(i) = C * x + D * u; % Output update
end

% ---- Proportional-Integral Controller (Kp = Ki = 1) ----
Kp = 1;
Ki = 1;
C_pi = tf([Kp, Ki], [1, 0]); % PI controller
Cd = c2d(C_pi, Ts, 'zoh');
[Ac, Bc, Cc, Dc] = ssdata(Cd);

% Initialize variables
y_pi = zeros(1, length(t)); % Output signal
xc = zeros(size(Ac, 1), 1); % Controller state
xg = zeros(size(A, 1), 1); % Motor state

% Simulation loop
for i = 1:length(t)
    if i == 1
        error = square_wave(i);
    else
        error = square_wave(i) - y_pi(i-1);
    end
    
    % PI controller update
    xc = Ac * xc + Bc * error;
    u = Cc * xc + Dc * error;
    
    % Motor update
    xg = A * xg + B * u;
    y_pi(i) = C * xg + D * u;
end

% ---- Plot Results ----
figure;
plot(t, square_wave, 'r', 'LineWidth', 1.5, 'DisplayName', 'Reference Signal');
hold on;
plot(t, y_p, 'b', 'LineWidth', 1.5, 'DisplayName', 'Proportional Controller Output');
plot(t, y_pi, 'g', 'LineWidth', 1.5, 'DisplayName', 'PI Controller Output');
grid on;
legend;
title('Steady-State Response Comparison: P vs PI Controller');
xlabel('Time (s)');
ylabel('Amplitude');



















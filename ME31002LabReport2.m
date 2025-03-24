%Q1
% Define system zeros and poles
zeros = [-2+2j; -2-2j];
poles = [0; -1];

% Create transfer function from zeros and poles
sys = zpk(zeros, poles, 1);

% Figure for root locus
figure(2);
rlocus(sys);
grid on;
title('Root Locus');
axis equal;


%Q2
% Define the Laplace variable 's'
s = tf('s');
% Define the system components
G1 = tf([100], [1,20]);       % First block
G2 = tf([10], [1,10,0]);  % Second block

% Case 1: Gc(s) = s
Gc1 = tf([1 0], 1);
C1 = feedback(G2, Gc1);% Closed-loop transfer function
T1 = feedback(G1* C1, 1);
L1 = G1 * G2 * Gc1;           % Open-loop transfer function for root locus

% Case 2: Gc(s) = s^2
Gc2 = tf([1 0 0], 1);
C2 = feedback(G2, Gc2);
T2 = feedback(G1* C2, 1);
L2 = G1 * G2 * Gc2;

% Case 3: Gc(s) = s^2 / (s + 20)
Gc3 = tf([1 0 0], [1 20]);
C3 = feedback(G2, Gc3);
T3 = feedback(G1 * C3, 1);
L3 = G1 * G2 * Gc3;

% Display the transfer functions
disp('Case 1 Transfer Function (T1):');
T1
disp('Case 2 Transfer Function (T2):');
T2
disp('Case 3 Transfer Function (T3):');
T3

% Plot root locus for each case
figure;
rlocus(L1);
xlabel('t(s)'); ylabel('y');
title('Root Locus: Case 1 (Gc(s) = s)');

figure;
rlocus(L2);
xlabel('t(s)'); ylabel('y');
title('Root Locus: Case 2 (Gc(s) = s^2)');

figure;
rlocus(L3);
xlabel('t(s)'); ylabel('y');
title('Root Locus: Case 3 (Gc(s) = s^2/(s+20))');

%Q3
% Define transfer functions of the system blocks
s = tf('s');
G1 = 1/(s+1);
G2 = 1/(2*s+1);
G3 = 1/(5*s+1);

% Combine plant transfer functions
plant = G1 * G2 * G3;

% Test different PID parameters
Kp_values = [0.5, 1, 2, 5, 10];
Ki_values = [0, 0.1, 0.3, 0.5, 1];
Kd_values = [0, 0.5, 1, 2, 5];

% Initialize variables to store the best parameters
best_Kp = 0;
best_Ki = 0;
best_Kd = 0;
best_settling_time = Inf;
best_overshoot = Inf;

% Create a figure for all responses
figure;
hold on;
grid on;
legends = {};

% Color map for plotting
colors = jet(length(Kp_values) * length(Ki_values) * length(Kd_values));
color_idx = 1;

% Loop through parameter combinations
for i = 1:length(Kp_values)
    for j = 1:length(Ki_values)
        for k = 1:length(Kd_values)
            Kp = Kp_values(i);
            Ki = Ki_values(j);
            Kd = Kd_values(k);
            
            % Create PID controller
            C = Kp + Ki/s + Kd*s;
            
            % Create closed-loop system
            sys_cl = feedback(C*plant, 1);
            
            % Get step response
            [y, t] = step(sys_cl, 50);
            
            % Calculate performance metrics
            info = stepinfo(y, t);
            rise_time = info.RiseTime;
            settling_time = info.SettlingTime;
            if isempty(settling_time), settling_time = 50; end
            overshoot = info.Overshoot;
            
            % Using a weighted combination of settling time and overshoot
            performance = 0.7*settling_time + 0.3*overshoot;
            best_performance = 0.7*best_settling_time + 0.3*best_overshoot;
            
            if performance < best_performance
                best_Kp = Kp;
                best_Ki = Ki;
                best_Kd = Kd;
                best_settling_time = settling_time;
                best_overshoot = overshoot;
            end
            
            % Plot only selected combinations to avoid too many lines
            if mod(color_idx, 5) == 0
                plot(t, y, 'Color', colors(color_idx,:));
                legends{end+1} = ['Kp=', num2str(Kp), ', Ki=', num2str(Ki), ', Kd=', num2str(Kd)];
            end
            color_idx = color_idx + 1;
        end
    end
end

title('Step Responses for Different PID Parameters');
xlabel('Time (seconds)');
ylabel('Amplitude');
legend(legends, 'Location', 'eastoutside');

% Display the best parameters
disp('Best PID Parameters:');
disp(['Kp = ', num2str(best_Kp)]);
disp(['Ki = ', num2str(best_Ki)]);
disp(['Kd = ', num2str(best_Kd)]);
disp(['Settling Time = ', num2str(best_settling_time)]);
disp(['Overshoot = ', num2str(best_overshoot), '%']);

% Plot the response with the best parameters
figure;
best_controller = best_Kp + best_Ki/s + best_Kd*s;
best_sys_cl = feedback(best_controller*plant, 1);
step(best_sys_cl, 50);
title(['Optimized Response: Kp=', num2str(best_Kp), ', Ki=', num2str(best_Ki), ', Kd=', num2str(best_Kd)]);
grid on;

%Q4
fib = @(n) round(((1+sqrt(5))/2)^n/sqrt(5));
fprintf('The answers are:\n');  
display([fib(3), fib(9), fib(15)])
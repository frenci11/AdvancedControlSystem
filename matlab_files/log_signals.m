

clc
% Assuming the logged data is stored in 'logsout'
% Extract the signals
out.logsout.getElement(2).Values % First input
out.logsout.getElement(3).Values % Second input
out.logsout.getElement(4).Values % Third input
out.logsout.getElement(1).Values

signal1.Data=squeeze(out.logsout.getElement(2).Values.Data);
signal1.Time=out.logsout.getElement(2).Values.Time;

signal2.Data=squeeze(out.logsout.getElement(3).Values.Data);

signal3.Data=squeeze(out.logsout.getElement(4).Values.Data);

signal4.Data=squeeze(out.logsout.getElement(1).Values.Data)';




% Plot the signals
figure(4);
clf

calculate_ylim = @(data) [min(data) - 0.1 * range(data), max(data) + 0.1 * range(data)];

subplot(4, 1, 1);
hold on;

plot(signal1.Time, signal1.Data(:, 1), 'Color', "#77AC30", 'DisplayName', 'x-d');
plot(signal1.Time, signal1.Data(:, 2), 'Color', "#0072BD", 'DisplayName', 'x',LineWidth=1);
plot(signal1.Time, signal1.Data(:, 3), 'Color', "black", 'DisplayName', 'x-t');
grid on;
grid minor;
hold off;
title('X');
ylim(calculate_ylim([signal1.Data(:, 2); signal1.Data(:, 2); signal1.Data(:, 1)])); % Adjust y-axis

xlabel('Time (s)');
ylabel('Amplitude');
legend show;

% Subplot 2: Signal 2
subplot(4, 1, 2);
hold on;
plot(signal1.Time, signal2.Data(:, 1), 'Color', "#77AC30", 'DisplayName', 'y-d');
plot(signal1.Time, signal2.Data(:, 2), 'Color', "#0072BD", 'DisplayName', 'y',LineWidth=1);
plot(signal1.Time, signal2.Data(:, 3), 'Color', "black", 'DisplayName', 'y-t');

grid on;
grid minor;
hold off;
title('Y');
ylim(calculate_ylim([signal2.Data(:, 2); signal2.Data(:, 2); signal2.Data(:, 1)])); % Adjust y-axis

xlabel('Time (s)');
ylabel('Amplitude');
legend show;


subplot(4, 1, 3);
hold on;

plot(signal1.Time, signal3.Data(:, 1), 'Color', "#77AC30", 'DisplayName', 'z-d');
plot(signal1.Time, signal3.Data(:, 2), 'Color', "#0072BD", 'DisplayName', 'z',LineWidth=1);
plot(signal1.Time, signal3.Data(:, 3), 'Color', "black", 'DisplayName', 'z-t');
grid on;
grid minor;
hold off;
title('Z');
ylim(calculate_ylim([signal3.Data(:, 1); signal3.Data(:, 1); signal3.Data(:, 1)])); % Adjust y-axis

xlabel('Time (s)');
ylabel('Amplitude');
legend show;

subplot(4, 1, 4);
hold on;
plot(signal1.Time, signal4.Data(:, 1), 'Color', "#0072BD", 'DisplayName', 'F_x',LineWidth=1);
plot(signal1.Time, signal4.Data(:, 2), 'Color', "#D95319", 'DisplayName', 'F_y',LineWidth=1);
plot(signal1.Time, signal4.Data(:, 3), 'Color', "#EDB120", 'DisplayName', 'F_z',LineWidth=1);

% plot(signal1.Time, signal4.Data(:, 4), 'Color', "#A2142F", 'DisplayName', 'force_x',LineWidth=1);
% plot(signal1.Time, signal4.Data(:, 5), 'Color', "#77AC30", 'DisplayName', 'force_y',LineWidth=1);
% plot(signal1.Time, signal4.Data(:, 6), 'Color', "#0072BD", 'DisplayName', 'force_z',LineWidth=1);
grid on;
grid minor;
hold off;
title('environment forces');
ylim(calculate_ylim([signal4.Data(:, 1); signal4.Data(:, 2); signal4.Data(:, 3)])); % Adjust y-axis

xlabel('Time (s)');
ylabel('Amplitude');
legend show;




% Adjust layout for clarity
sgtitle('admittance control');

exportgraphics(gcf, 'simulation_results.pdf', 'ContentType', 'vector');

%%
clear t data

% Extract trajectory data
fprintf('RETREIVING DATA FROM');out.logsout.getElement(1)
data = out.logsout.getElement(1).Values.Data; % Extract [x, y, z]
data = squeeze(data)'; % Transpose to size [10001 x 3]
t = out.logsout.getElement(1).Values.Time; % Time vector

downsample_factor = 100;
data = data(1:downsample_factor:end, :); % Take every 10th sample
t = t(1:downsample_factor:end); % Downsample time vector

% Real-time 3D plot
c=figure(1);
subplot(2,1,2);
hold on
for i = 1:length(t)
    % Plot trajectory up to the current time step
    plot3(data(1:i, 1), data(1:i, 2), data(1:i, 3), 'b', 'LineWidth', 1); % Trajectory path
    %hold on;
    scatter3(data(i, 1), data(i, 2), data(i, 3), 30, 'r', 'filled'); % Current point

end
hold off

%myAxes=findobj(c,'Type','Axes')
exportgraphics(gcf, 'robot trajectory.pdf', 'ContentType', 'image');












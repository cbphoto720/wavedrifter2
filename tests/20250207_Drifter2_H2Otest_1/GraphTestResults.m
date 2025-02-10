close all; clear all; clc
addpath(genpath('C:\Users\Cars\Documents\GitHub\wavedrifter2\tests\20250207_Drifter2_H2Otest_1'));

%%
% read the cleaned file

inputFile = '51control.log';  % Replace with your actual input file
control=parseCleanedFile(inputFile);

inputFile = '54wet_float.log';  % Replace with your actual input file
float=parseCleanedFile(inputFile);

inputFile = '55wet_submerged-surface.log';  % Replace with your actual input file
Subsurface=parseCleanedFile(inputFile);

inputFile = '56wet_submerged-DEEP.log';  % Replace with your actual input file
underwater10=parseCleanedFile(inputFile);

inputFile = 'LASTwet_submerged-DEEPBLUE.log';  % Replace with your actual input file
underwater25=parseCleanedFile(inputFile);


%% Plot a single histogram
figure(1)
x=[control.RSSI];
histogram(x)

% Set figure size
set(0,'units','pixels');
scr_siz = get(0,'ScreenSize');
set(gcf,'Position',[floor([10 150 scr_siz(3)*0.4 scr_siz(4)*0.5])]);

% Add title and axis labels
title('Histogram of RSSI Values');        % Title of the plot
xlabel('RSSI (dBm)');                           % X-axis label
ylabel('Frequency');                      % Y-axis label

%%

%% Convert struct fields to variables with proper names
Control_RSSI = [control.RSSI].';  % Convert RSSI from control struct to a variable
Float_RSSI = [float.RSSI].';      % Convert RSSI from float struct to a variable
Subsurface_RSSI = [Subsurface.RSSI].';  % Convert RSSI from subsurface struct to a variable
Underwater10_RSSI = [underwater10.RSSI].';  % Convert RSSI from underwater10 struct to a variable
Underwater25_RSSI = [underwater25.RSSI].';  % Convert RSSI from underwater25 struct to a variable

%% Toggle variables for inclusion in plot (set to true or false)
includeControl = true;    % Set to false to exclude 'Control' data
includeFloat = true;      % Set to false to exclude 'Floating' data
includeSubsurface = false; % Set to false to exclude 'Subsurface' data
includeUnderwater10 = false; % Set to false to exclude 'Underwater10mm' data
includeUnderwater25 = true; % Set to false to exclude 'Underwater25mm' data

%% Dynamically calculate the minimum length based on included datasets
selected_lengths = [];
if includeControl
    selected_lengths = [selected_lengths, length(Control_RSSI)];
end
if includeFloat
    selected_lengths = [selected_lengths, length(Float_RSSI)];
end
if includeSubsurface
    selected_lengths = [selected_lengths, length(Subsurface_RSSI)];
end
if includeUnderwater10
    selected_lengths = [selected_lengths, length(Underwater10_RSSI)];
end
if includeUnderwater25
    selected_lengths = [selected_lengths, length(Underwater25_RSSI)];
end

% Calculate the new min_len based on selected datasets
min_len = min(selected_lengths);

%% Truncate all variables to the minimum length based on selected datasets
if includeControl
    TRUNC_Control_RSSI = Control_RSSI(1:min_len);  % Truncate control RSSI data
end
if includeFloat
    TRUNC_Float_RSSI = Float_RSSI(1:min_len);      % Truncate float RSSI data
end
if includeSubsurface
    TRUNC_Subsurface_RSSI = Subsurface_RSSI(1:min_len);  % Truncate subsurface RSSI data
end
if includeUnderwater10
    TRUNC_Underwater10_RSSI = Underwater10_RSSI(1:min_len);  % Truncate underwater10 RSSI data
end
if includeUnderwater25
    TRUNC_Underwater25_RSSI = Underwater25_RSSI(1:min_len);  % Truncate underwater25 RSSI data
end

%% Now, plot the truncated data

% Define the range of RSSI values of interest
minRSSI = -110;   % Minimum value of RSSI
maxRSSI = -95;    % Maximum value of RSSI

% Set consistent bin edges for all histograms
binEdges = linspace(minRSSI, maxRSSI, abs(minRSSI-maxRSSI)); % Adjust number of bins if needed

figure(2)

% Define histogram features
global_alpha= 0.85;
% colors = {'#9962a7','#72a49c','#6ba9a0','#c94f3a','#152852'};
colors = {'#c94f3a','#72a49c','#658786','#456977','#3b64a3'}; % Blue sequential
% colors = {'#9962a7','#72a49c','#6ba9a0','#c94f3a','#152852'};
alpha = {1,global_alpha,global_alpha,global_alpha,global_alpha};
normalization_mode='probability';

% Plot each dataset with consistent bin edges, different colors, transparency, and normalization
if includeControl
    histogram(Control_RSSI, 'BinEdges', binEdges, 'FaceColor', colors{1}, 'FaceAlpha', alpha{1}, 'EdgeColor', 'none', 'Normalization', normalization_mode);   % Control - Light Green
    hold on;
end

if includeFloat
    histogram(Float_RSSI, 'BinEdges', binEdges, 'FaceColor', colors{2}, 'FaceAlpha', alpha{2}, 'EdgeColor', 'none', 'Normalization', normalization_mode);     % Floating - Light Blue
    hold on;
end

if includeSubsurface
    histogram(Subsurface_RSSI, 'BinEdges', binEdges, 'FaceColor', colors{3}, 'FaceAlpha', alpha{3}, 'EdgeColor', 'none', 'Normalization', normalization_mode); % Subsurface - Green
    hold on;
end

if includeUnderwater10
    histogram(Underwater10_RSSI, 'BinEdges', binEdges, 'FaceColor', colors{4}, 'FaceAlpha', alpha{4}, 'EdgeColor', 'none', 'Normalization', normalization_mode); % Underwater10 - Cyan
    hold on;
end

if includeUnderwater25
    histogram(Underwater25_RSSI, 'BinEdges', binEdges, 'FaceColor', colors{5}, 'FaceAlpha', alpha{5}, 'EdgeColor', 'none', 'Normalization', normalization_mode); % Underwater25 - Dark Blue
end

% Set figure size
set(0, 'units', 'pixels');
scr_siz = get(0, 'ScreenSize');
scalefactor=1.5;
set(gcf, 'Position', [floor([10 150 900*scalefactor 750*scalefactor])]);

% Add title and axis labels
fontsize=24;
fontsize_axis=fontsize*0.85;
title('Drifter Broadcast Signal Strength in Freshwater (Low-Power mode, 15m away)', 'FontSize', fontsize); % Title
xlabel('RSSI (dBm)', 'FontSize', fontsize_axis);   % X-axis label
ylabel('Relative probability', 'FontSize', fontsize_axis);    % Y-axis label (normalized frequency)

% Add legend conditionally
legendEntries = {};
if includeControl
    legendEntries{end+1} = 'Control (air)';
end
if includeFloat
    legendEntries{end+1} = 'Floating (+20mm)';
end
if includeSubsurface
    legendEntries{end+1} = 'Subsurface';
end
if includeUnderwater10
    legendEntries{end+1} = 'Underwater (-10mm)';
end
if includeUnderwater25
    legendEntries{end+1} = 'Underwater (-25mm)';
end
legend(legendEntries, 'FontSize', fontsize_axis);

hold off;  % Release the hold on the plot

%%
% % differnece checker of UTC
figure(3)
% x=[ControlBasedata.lastUTC];
x= [control.lastUTC].';

x2 = x;  % Make a copy of x
x2(1) = x(1) - mean(x);  % First value: subtract the mean of x
x2(2:end) = diff(x);  % Subtract each value from the previous one

plot(x2)

%%




% Plot the voltage




%% Convert struct fields to variables with proper names for voltage
Control_voltage = [control.voltage].';  % Convert voltage from control struct to a variable
Float_voltage = [float.voltage].';      % Convert voltage from float struct to a variable
Subsurface_voltage = [Subsurface.voltage].';  % Convert voltage from subsurface struct to a variable
Underwater10_voltage = [underwater10.voltage].';  % Convert voltage from underwater10 struct to a variable
Underwater25_voltage = [underwater25.voltage].';  % Convert voltage from underwater25 struct to a variable

%% Toggle variables for inclusion in plot (set to true or false)
includeControl = true;    % Set to false to exclude 'Control' data
includeFloat = true;      % Set to false to exclude 'Floating' data
includeSubsurface = true; % Set to false to exclude 'Subsurface' data
includeUnderwater10 = true; % Set to false to exclude 'Underwater10mm' data
includeUnderwater25 = true; % Set to false to exclude 'Underwater25mm' data

%% Now, plot the raw voltage data (without truncation)

figure(6)

% Define the colors and legend names
colors = {'#c94f3a','#72a49c','#658786','#456977','#3b64a3'}; % Custom color palette
legendNames = {'Control (air)', 'Floating (+20mm)', 'Subsurface', 'Underwater10mm', 'Underwater (-25mm)'};
alpha = {1, 0.85, 0.85, 0.85, 0.85};  % Alpha transparency for each dataset

% Create the plot for each dataset
hold on;

time = 1:max([length(Control_voltage), length(Float_voltage), length(Subsurface_voltage), length(Underwater10_voltage), length(Underwater25_voltage)]);

if includeControl
    plot(time(1:length(Control_voltage)), Control_voltage, 'Color', colors{1}, 'LineWidth', 1);   % Control - Red
end

if includeFloat
    plot(time(1:length(Float_voltage)), Float_voltage, 'Color', colors{2}, 'LineWidth', 1);     % Floating - Teal
end

if includeSubsurface
    plot(time(1:length(Subsurface_voltage)), Subsurface_voltage, 'Color', colors{3}, 'LineWidth', 1);  % Subsurface - Greenish Blue
end

if includeUnderwater10
    plot(time(1:length(Underwater10_voltage)), Underwater10_voltage, 'Color', colors{4}, 'LineWidth', 1);  % Underwater10 - Dark Green
end

if includeUnderwater25
    plot(time(1:length(Underwater25_voltage)), Underwater25_voltage, 'Color', colors{5}, 'LineWidth', 1);  % Underwater25 - Blue
end

% Set figure size
set(0, 'units', 'pixels');
scr_siz = get(0, 'ScreenSize');
set(gcf, 'Position', [floor([10 150 1000 750])]);

% Add title and axis labels
title('Raw Voltage Values vs Time for Different Data Sources'); % Title
xlabel('Time (Index)');   % X-axis label
ylabel('Voltage (V)');    % Y-axis label

% Add legend conditionally
legendEntries = {};
if includeControl
    legendEntries{end+1} = 'Control (air)';
end
if includeFloat
    legendEntries{end+1} = 'Floating (+20mm)';
end
if includeSubsurface
    legendEntries{end+1} = 'Subsurface';
end
if includeUnderwater10
    legendEntries{end+1} = 'Underwater10mm';
end
if includeUnderwater25
    legendEntries{end+1} = 'Underwater (-25mm)';
end
legend(legendEntries);

hold off;  % Release the hold on the plot

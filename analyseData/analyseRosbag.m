clear all;
close all;
clc;
format long;

%% Read Data

%Load Data
bag = rosbag('cam1_info_25hz_50dc_2019-04-29-23-49-48.bag');
%Load Timestamps
TimeStampsTable = bag.MessageList(:,1);
TimeStamps = table2array(TimeStampsTable);

SetFreq = 25;


%% Manipulate Data
TimeStamps = TimeStamps - bag.StartTime;
dt = diff(TimeStamps);
freq = 1./dt;

%% Analyse Data
MinFreq = min(freq);
MaxFreq = max(freq);
MeanFreq = mean(freq);
VarFreq = var(freq);

%% Plot Data
figure(1);
histogram(freq,500);
hold on;
line([25 25], ylim, 'LineWidth', 2, 'Color', 'green');
hold on;
line([MeanFreq MeanFreq], ylim, 'LineWidth', 2, 'Color', 'red');
hold on;
line([MeanFreq-VarFreq MeanFreq-VarFreq], ylim, 'LineWidth', 2, 'Color', 'magenta');
hold on;
line([MeanFreq+VarFreq MeanFreq+VarFreq], ylim, 'LineWidth', 2, 'Color', 'magenta');
hold on;
line([MinFreq MinFreq], ylim, 'LineWidth', 2, 'Color', [0.4940, 0.1840, 0.5560]);
hold on;
line([MaxFreq MaxFreq], ylim, 'LineWidth', 2, 'Color', [0.3010, 0.7450, 0.9330]);

legend(['Histogram'],['set frequency: ' num2str(SetFreq)],['mean freq: ' num2str(MeanFreq)],...
    ['variance freq: ' num2str(MeanFreq-VarFreq)], ['variance freq: ' num2str(MeanFreq+VarFreq)],...
    ['Min freq: ' num2str(MinFreq)], ['Max freq: ' num2str(MaxFreq)]);


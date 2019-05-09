clear all;
close all;
clc;
format long;

%% Read Data

%Load Data
bag = rosbag('2019-05-08-08-26-44.bag');
allMsgs = readMessages(bag,"DataFormat","struct");

%Load Timestamps
n = size(allMsgs,1);
for i=1:n
   TimeStamps(i,1) = double(allMsgs{i}.Header.Stamp.Sec) + double(allMsgs{i}.Header.Stamp.Nsec) * 1e-9;
end

%TimeStampsTable = bag.MessageList(:,1);
%TimeStamps = table2array(TimeStampsTable);

SetFreq = 25;


%% Manipulate Data
%TimeStamps = TimeStamps - bag.StartTime;
TimeStamps = TimeStamps - TimeStamps(1);
dt = diff(TimeStamps);
freq = 1./dt;


%% Ground Truth Data
nStamps = size(TimeStamps,1);
TimeStampsGroundTruth = ones(nStamps,1);
TimeStampsGroundTruth(1) = 0;
TimeStampsGroundTruth(2:end) = 1/SetFreq;
TimeStampsGroundTruth = cumsum(TimeStampsGroundTruth);


%% Analyse Data (Frequency)
MinFreq = min(freq);
MaxFreq = max(freq);
MeanFreq = mean(freq);
VarFreq = var(freq);


%% Analyse Data (Jitter)
DiffVec = (TimeStamps - TimeStampsGroundTruth).*1000;
MeanDiff = mean(DiffVec);
VarDiff = var(DiffVec);
MinDiff = min(DiffVec);
MaxDiff = max(DiffVec);


%% Plot Data
figure(1);
histogram(freq,500);
hold on;
line([SetFreq SetFreq], ylim, 'LineWidth', 2, 'Color', 'green');
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



figure(2);
histogram(DiffVec,500);
xlabel('Jitter [ms]');
hold on;
line([0 0], ylim, 'LineWidth', 2, 'Color', 'green');
hold on;
line([MeanDiff MeanDiff], ylim, 'LineWidth', 2, 'Color', 'red');
hold on;
line([MeanDiff-VarDiff MeanDiff-VarDiff], ylim, 'LineWidth', 2, 'Color', 'magenta');
hold on;
line([MeanDiff+VarDiff MeanDiff+VarDiff], ylim, 'LineWidth', 2, 'Color', 'magenta');
hold on;
line([MinDiff MinDiff], ylim, 'LineWidth', 2, 'Color', [0.4940, 0.1840, 0.5560]);
hold on;
line([MaxDiff MaxDiff], ylim, 'LineWidth', 2, 'Color', [0.3010, 0.7450, 0.9330]);

legend(['Histogram'],['optimum: ' num2str(0)],['mean Jitter: ' num2str(MeanDiff)],...
    ['variance Jitter: ' num2str(MeanDiff-VarDiff)], ['variance Jitter: ' num2str(MeanDiff+VarDiff)],...
    ['Max negative Jitter: ' num2str(MinDiff)], ['Max positive Jitter: ' num2str(MaxDiff)]);

figure(3);
subplot(2,1,1);
plot(TimeStamps,DiffVec);
xlabel('Time [s]');
ylabel('Jitter [ms]');
subplot(2,1,2);
plot(TimeStamps(2:end),dt);
xlabel('Time [s]');
ylabel('dt [ms]');




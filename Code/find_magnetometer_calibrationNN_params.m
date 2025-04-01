clc; clear; close all;

% Define the neural network structure
hiddenLayers = [32, 16];

% Allocate arrays for storage
XY_all = []; heading_all = [];

% Define the data sets to be used in NN training
calibFiles = "../trainingData/" + ["calib1_rotate.mat", "calib2_straight.mat", "task1_1.mat", "task1_2.mat", "task2_1.mat", "task2_2.mat", "task2_3.mat"];

% Load and Process All Datasets

for fileIdx = 1:length(calibFiles)
    fprintf("Loading %s ...\n", calibFiles(fileIdx));
    data = load(calibFiles(fileIdx));

    % Extract ground truth quaternion and time
    q_gt  = squeeze(data.out.GT_rotation.signals.values);
    t_gt  = squeeze(data.out.GT_rotation.time);
    yaw   = quat2eul(q_gt, 'ZXY'); 
    yaw   = unwrap(yaw(:,1));

    % Extract magnetometer data
    Mraw   = squeeze(data.out.Sensor_MAG.signals.values)';
    M_time = squeeze(data.out.Sensor_MAG.time)';

    % Combine the 2D measurements
    Mx     = Mraw(:,2);
    My     = Mraw(:,3);
    XY     = [Mx, My];

    % Store all of the Data
    XY_all = [XY_all; XY];
    heading_all = [heading_all; [cos(yaw), sin(yaw)]];
end

fprintf("Total samples loaded: %d\n", size(XY_all, 1));

% Normalize Input
mu = mean(XY_all);
sigma = std(XY_all);
X_norm = (XY_all - mu) ./ sigma;

% Split Train/Test
N = size(X_norm, 1);
idx = randperm(N);
split = round(0.9 * N);

X_train = X_norm(idx(1:split), :)';
Y_train = heading_all(idx(1:split), :)';

X_test  = X_norm(idx(split+1:end), :)';
Y_test  = heading_all(idx(split+1:end), :)';

% Train Neural Network
net = feedforwardnet(hiddenLayers);
net = configure(net, X_train, Y_train);
net.trainParam.showWindow = true;
net.trainParam.epochs = 1000;
net.trainParam.max_fail = 10;

[net, tr] = train(net, X_train, Y_train);

%% Save Model
NNMagCal.net = net;
NNMagCal.mu  = mu;
NNMagCal.sigma = sigma;
save('NNMagCal_2D.mat', 'NNMagCal');  % Save all in one .mat file

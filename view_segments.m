% Specify the path to the PCD file
pcdFilePath = 'rosbags/processed/2023-11-25_planning_intersection_1_matlab/pcd_top_lidar/1.pcd';

% Read the PCD file
ptCloud = pcread(pcdFilePath);

% Load the labels (assuming they are stored in a separate file)
labelsFilePath = 'rosbags/processed/2023-11-25_planning_intersection_1_matlab/pcd_top_lidar/1.mat';
load(labelsFilePath);

% Color the point cloud according to the labels
colors = squeeze(label2rgb(labels_final));
ptCloud.Color = colors;

% Show the colored point cloud
pcshow(ptCloud);

% configs
rosbag_folder = 'rosbags/raw';
rosbag_name = '2023-10-28-calibration-bags/cam_lidar_oct_28_trial2';
rosbag_path = fullfile(rosbag_folder, rosbag_name);

max_frames = 30;
slop_s = 0.1;
processed_data_folder = 'rosbags/processed';
extracted_bag_name = '2023-10-28-calibration-bags/cam_lidar_oct_28_trial2';
force_extract_LC_pairs = true;
processed_data_path = fullfile(processed_data_folder, extracted_bag_name);
img_path = fullfile(processed_data_folder, extracted_bag_name, 'images_center_wide');
mask_path = fullfile(processed_data_folder, extracted_bag_name, 'images_center_wide_mask');
proc_mask_path = fullfile(processed_data_folder, extracted_bag_name, 'images_center_wide_mask_processed');
lidar_path = fullfile(processed_data_folder, extracted_bag_name, 'pcd_top_lidar');

% camera intrinsics
focalLength = [962.862149708933, 962.223752837268];
principalPoint = [820.764326395683, 532.401009233272];
imageSize = [1100, 1604];
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% register custom ROS message types
custom_msg_path = fullfile(pwd, 'generated_msgs');
ros2RegisterMessages(custom_msg_path);
% gt extrinsics
gt_c2l = [0.0209356883783334,	-0.999780806006143,	0.000192077675366802,	-0.104702326586575;
        -0.0265626680739577,	-0.000748281607108440,	-0.999646870019323,	2.11403030646542;
        0.999427897157628,	0.0209231932635657,	-0.0265725114872007,	0.315480514345691];
gt_c2l = rigidtform3d(gt_c2l);

% initial guess
init_guess = [0, -1, 0, 0;
              0, 0, -1, 2;
              1, 0, 0, 0];
gt_c2l = rigidtform3d(gt_c2l);

% process raw rosbag
if ~exist(processed_data_path, 'dir') || force_extract_LC_pairs
    "begin processing"
    extract_LC_pairs(rosbag_path, img_path, mask_path, proc_mask_path, lidar_path);
    "finished processing"
end

% iterate through each image in img_path
img_files = dir(fullfile(img_path, '*.png'));
% sort the files
img_files = natsortfiles({img_files.name});
for i = 1:length(img_files)
    img_file = img_files{i};
    img = imread(fullfile(img_path, img_file));
    pc = pcread(fullfile(lidar_path, [img_file(1:end-4), '.pcd']));
    
    [imPts, indices] = projectLidarPointsOnImage(pc, intrinsics, gt_c2l);

    % visualize
    figure(1)
    clf;
    imshow(img)
    hold on;
    plot(imPts(:, 1), imPts(:, 2), 'r.')
    hold off;
    % axis equal;
    % title('Ground Truth (red) vs. Estimated (blue)');
    drawnow
end



% visualize current best guess

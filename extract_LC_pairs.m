% TODO
% extract time synch pairs
% run SAM to get masks
% lidar preprocess:
%  - remove outliers
%  - compute surface normals
%  - segment planes and clusters

function [processed_imgs, processed_pc] = extract_LC_pairs(rosbag_path, img_path, mask_path, proc_mask_path, lidar_path)
    % make folders if they don't exist
    if ~exist(img_path, 'dir')
        mkdir(img_path);
    end
    if ~exist(mask_path, 'dir')
        mkdir(mask_path);
    end
    if ~exist(proc_mask_path, 'dir')
        mkdir(proc_mask_path);
    end
    if ~exist(lidar_path, 'dir')
        mkdir(lidar_path);
    end
    bag = ros2bagreader(rosbag_path);
    camSel = bag.select('Topic', '/camera/center_wide');
    lidarSel = bag.select('Topic', '/lidar/center/top');
    camMsgs = readMessages(camSel);
    camMsgsArray = [camMsgs{:}];
    lidarMsgs = readMessages(lidarSel);
    lidarMsgsArray = [lidarMsgs{:}];

    % extract time stamps from messages
    % initialize empty arrays of ros2time objects
    camTime= zeros(1, length(camMsgs), 'int32');
    lidarTime= zeros(1, length(lidarMsgs), 'int32');
    
    camHeaders = [camMsgsArray.header];
    lidarHeaders = [lidarMsgsArray.header];
    camStamps = [camHeaders.stamp];
    lidarStamps = [lidarHeaders.stamp];
    camSecs = [camStamps.sec];
    lidarSecs = [lidarStamps.sec];
    camNanosecs = [camStamps.nanosec];
    lidarNanosecs = [lidarStamps.nanosec];
    
    diffSecs = camSecs - lidarSecs'; % (num lidar, num cam)
    % convert ns to ms to avoid overflow
    camMs = camNanosecs / 1e6;
    lidarMs = lidarNanosecs / 1e6; 
    % cast to int32 to avoid overflow
    camMs = int32(camMs);
    lidarMs = int32(lidarMs);

    diffMs = camMs - lidarMs'; % (num lidar, num cam)
    diffTimeMS = diffSecs * 1e3 + diffMs; % (num lidar, num cam)
    diffTimeMS = abs(diffTimeMS);

    [minDiff, minIdx] = min(diffTimeMS, [], 2);

    % take top k time diffs
    [diffs, idx] = sort(minDiff);
    k = 30; % TODO convert to parameter
    idx = idx(1:k);
    disp("time diffs in ms " + diffs(k));
    for i = 1:k
        % PC preprocessing, TODO move to a separate function
        pc_xyz = rosReadXYZ(lidarMsgs{idx(i)});
        pc_intensity = rosReadField(lidarMsgs{idx(i)}, 'intensity');
        pc_raw = pointCloud(pc_xyz, "Intensity", pc_intensity);

        % denoise
        pc_denoised = pcdenoise(pc_raw);

        % compute surface normals
        pc_normals = pcnormals(pc_denoised);

        pc_denoised = pointCloud(pc_denoised.Location, 'Intensity', pc_denoised.Intensity, 'Normal', pc_normals);
        % TODO downsample

        % segment planes, TODO convert to configurable parameters
        [groundPtsIdx,nonGroundPtCloud,groundPtCloud] = segmentGroundSMRF(pc_denoised);
        nonGroundPtCloud = removeInvalidPoints(nonGroundPtCloud);
        groundPtCloud = removeInvalidPoints(groundPtCloud);
        % pc_with_class = {};
        % numPlanes = 0; % could replace with length(pc_with_class)
        % pc_outlier = copy(pc_denoised);
        % TODO convert to parameter
        % while numPlanes < 10
        %     [model, inlier, outlier] = pcfitplane(pc_outlier, 0.1, [0, 0, 1], 0.1);

        %     if length(inlier) < 1000 % TODO set as parameter
        %         break
        %     end

        %     pc_in = select(pc_outlier, inlier);
        %     pc_in.Color = [0, 0, 1];
        %     numPlanes = numPlanes + 1;
        %     pc_with_class{numPlanes} = pc_in;
        %     pc_outlier = select(pc_outlier, outlier).copy();
        % end

        % segment clusters, TODO convert to configurable parameters
        [labels, numClusters] = pcsegdist(nonGroundPtCloud, 0.1, "NumClusterPoints", 10);
        groundLabels = zeros(length(groundPtCloud.Location), 1) + numClusters + 1;
        
        % combine point clouds
        % pc_final = pccat([groundPtCloud, nonGroundPtCloud]);
        % labels_final = [groundLabels; labels];
        pc_final = nonGroundPtCloud;
        labels_final = labels;

        % image preprocessing, TODO move to a separate function
        imgIdx = minIdx(idx(i));
        img = rosReadImage(camMsgs{imgIdx});
        % save image and point cloud
        img_name = sprintf('%s/%d.png', img_path, i)
        imwrite(img, img_name)
        lidar_name = sprintf('%s/%d.pcd', lidar_path, i)
        pcwrite(pc_final, lidar_name)
        % save labels
        labels_name = sprintf('%s/%d.mat', lidar_path, i);
        save(labels_name, 'labels_final');
    end

    % run SAM to get masks
%{
    python scripts/amg.py --checkpoint sam_vit_l_0b3195.pth --model-type vit_l --input ./data/kitti/000000/images/  --output ./data/kitti/000000/masks/ --stability-score-thresh 0.9 --box-nms-thresh 0.5 --stability-score-offset 0.9
%}

    % SAM_params = sprintf("/home/brian/Documents/School/AER1515/Project/segment-anything/scripts/amg.py --checkpoint /home/brian/Documents/School/AER1515/Project/segment-anything/ckpt/sam_vit_l_0b3195.pth --model-type vit_l --input %s --output %s --stability-score-thresh 0.9 --box-nms-thresh 0.5 --stability-score-offset 0.9", img_path, mask_path);
    % pyrunfile(SAM_params);
    % procmask_params = sprintf("/home/brian/Documents/School/AER1515/Project/CalibAnything/processed_mask.py -i %s -o %s", mask_path, proc_mask_path);
    % pyrunfile(procmask_params);
    % load in point clouds and image masks
end

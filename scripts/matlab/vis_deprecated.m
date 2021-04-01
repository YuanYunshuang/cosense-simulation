% clear all
% close all

%% Paths
path = "../data/simulation/j509e669";
ego = "000669";
% participants 
vehicles = dir2(path);
vehicles = extractfield( vehicles([vehicles.isdir]==1), 'name');
% Point cloud paths
lidar_path = path + filesep + ego +filesep + "lidar_sem";
lidars = dir2(lidar_path);
lidars_data = lidars(contains({lidars.name}, 'pcd'));
lidars_meta = lidars(contains({lidars.name}, 'txt'));
% Trajectory
traj = read_info(path + filesep + "info.csv");
% Get frames in cells
frames = keys(traj);

%% visualize simulation data
% set up players
player1 = pcplayer([-200 0],[-100 100],[0 5], 'MarkerSize', 8);
% player2 = pcplayer([-200 0],[-100 100],[0 5], 'MarkerSize', 5);

while isOpen(player1) 
    for i=1:(length(lidars_data))
%      pcfile_ego = fullfile(lidars_data(i).folder, lidars_data(i).name);
      frame = lidars_data(i).name(1:end-4);
%      metafile_ego = fullfile(lidars_data(i).folder, frame + "_meta.txt");
%      ptCloud_ego = pcread(pcfile_ego);
%      [eframe, etimestamp, etform] = read_meta(metafile_ego);
%      pcCloud_ego = pcdownsample(ptCloud_ego,'gridAverage',0.2);

     neighbor_index = {}; 
     ptClouds = pointCloud.empty;
     tforms = rigid3d.empty;
     for j=1:length(vehicles)
         id = vehicles{j};
         filename = path + filesep + id + filesep + "lidar_sem" ...
             + filesep + frame + ".pcd";
         if isfile(filename)
             neighbor_index{end+1} = id;
             ptc = pcread(filename);
             pcdownsample(ptc,'gridAverage',0.2);
             ptClouds(end+1) = ptc;             
             meta_file = extractBetween(filename, 1, strlength(filename)-4) + "_meta.txt";
             [f, timestamp, tform] = read_meta(meta_file);
             tforms(end + 1) = tform;
         end
     end
     
     if length(neighbor_index)>0
        fprintf('frame: %06s, vehicles: %d \n', frame, length(neighbor_index));
        ptClouds_aligned = pcalign(ptClouds, tforms, 1);
        view(player1,ptClouds_aligned); 
%         view(player2,pctransform(ptCloud_ego,etform));
     end
     %subplot(2,1,1); pcshow(ptCloud);
      
     
    end
end 
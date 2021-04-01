clear all;

%% Prepare data for visualization
% read trajectory data
data = load('trajectory_j509_e532.mat').data;
% vals = values(data);
% for id=1:data.Count
%     size(vals{id})
% end
% path
path = "../data/simulation/j509e532/000532";

%% visualize simulation data
player1 = pcplayer([-200 0],[-100 100],[0 5], 'MarkerSize', 5);
player2 = pcplayer([-200 0],[-100 100],[0 5], 'MarkerSize', 5);
pcs1 = dir2(path + filesep + "lidar_sem");
pause(5);
while isOpen(player1) 
    for i=1:(length(pcs1)-1)
     ptClouds = pointCloud.empty;
     tforms = rigid3d.empty;
     ptCloud_ego = pcread(fullfile(pcs1(i).folder, pcs1(i).name));
     % pcCloud_ego = pcdownsample(ptCloud_ego,'gridAverage',0.1);
     frame = pcs1(i).name(1:end-4);
     neighbor_index = {};
     visible_vehicles = data(frame);
     
     n = size(visible_vehicles);
     for j=1:n(1)
         id = sprintf('%06d', visible_vehicles(j, 1));
         filename = fileparts(path) + filesep + id + filesep + "lidar_sem" ...
             + filesep + frame + ".pcd";
         if isfile(filename)
             neighbor_index{end+1} = id;
             ptc = pcread(filename);
             ptClouds(end+1) = ptc;
             % pcdownsample(pct,'gridAverage',0.1);
             loc =  visible_vehicles(j, 2:4);
             loc(3) = loc(3) + 1.73;
             rotmat = eul2rotm(visible_vehicles(j, 5:7) / 180 * pi);
             tform = rigid3d(rotmat, loc);
             if id=="000532"
                 tform_ego = tform;
             else
                 tform_ego = 0;
             end
             tforms(end + 1) = tform;
         end
     end
     ptClouds_aligned = pcalign(ptClouds, tforms, 1);
     if length(neighbor_index)>0
        fprintf('frame: %06s, vehicles: %d \n', frame, length(neighbor_index));
        neighbor_index
     end
     %subplot(2,1,1); pcshow(ptCloud);
     view(player1,ptClouds_aligned);  
     view(player2,pctransform(ptCloud_ego,tform_ego));
    end
end 
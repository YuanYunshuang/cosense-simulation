path = "\\koko\tmp\yunshuang\data\simulation\j509e669\000669";
fid = fopen('\\koko\tmp\yunshuang\data\simulation\j509e532\000460\lidar');
%% read kitti binary file
% points = fread(fid,'float');
% n_points = length(points) / 4;
% points = reshape(points, [n_points, 4]);
% max(points(:, 1))

%% visualize simulation data
player1 = pcplayer([-80 80],[-80 80],[-2 2], 'MarkerSize', 10);
pcs1 = dir2(path + "\lidar");
player2 = pcplayer([-80 80],[-80 80],[-2 2], 'MarkerSize', 10);
pcs2 = dir2(path + "\lidar_sem");
pause(5);
while isOpen(player1) 
    for i=1:(length(pcs1)-1)
     ptCloud = pcread(fullfile(pcs1(i).folder, pcs1(i).name));
     %subplot(2,1,1); pcshow(ptCloud);
     view(player1,ptCloud);
     ptCloud = pcread(fullfile(pcs2(i).folder, pcs2(i).name));
     %subplot(2,1,2); pcshow(ptCloud);
     view(player2,ptCloud);
     % ptCloud = pcmerge(ptCloud1, ptCloud2, 1);
     %drawnow; 
     
    end
end 

% 
% pcs = dir('F:\multiview_ego1261\001261\camera');
% for i=1:length(pcs)
%  ptCloud = pointCloud(pcread(pcs(i)));
%  pcshow(ptCloud);
%  pause(0.5);
% end
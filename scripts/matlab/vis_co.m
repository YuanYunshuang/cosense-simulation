clear all
close all

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
% set up figure
% player1 = pcplayer([-200 0],[-100 100],[0 5], 'MarkerSize', 8);
limits = [-200, 0,-100, 100,-2, 6];
makersize = 2;
filter = 0.5;
set(0,'defaultfigurecolor',[1,1,1])
fig = figure;
set(gcf,'position',[0,0,1200,1000])
xlim([-200 0])
xlabel('X');
ylim([-100 100])
ylabel('Y');
zlim([-2 6])
zlabel('Z');
axis vis3d
set(gca,'Color','k');
set(gca,'DataAspectRatio',[1 1 0.5])
view(-33,42);  
hold on

writerObj = VideoWriter('j509e669_co.avi', 'Motion JPEG AVI');
writerObj.FrameRate = 10;
writerObj.Quality= 100;
open(writerObj);
 
%% Loop
try
    for i=1:(length(lidars_data))
         frame = lidars_data(i).name(1:end-4);

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
                 pcdownsample(ptc,'random',filter);
                 ptClouds(end+1) = ptc;             
                 meta_file = extractBetween(filename, 1, strlength(filename)-4) + "_meta.txt";
                 [f, timestamp, tform] = read_meta(meta_file);
                 tforms(end + 1) = tform;
             end
         end
        % plot
        if length(neighbor_index)>0
            text = sprintf('frame: %06s, vehicles: %d', frame, length(neighbor_index));
            delete(findall(gcf,'type','annotation'))
            annotation('textbox', [0.05, 0.85, 0.35, 0.06], 'string', text,'FitBoxToText','on')
            ptClouds_aligned = pcalign(ptClouds, tforms);
            coors = ptClouds_aligned.Location;
            cmp = jet(256);
            inds = round((coors(:,3) + 1)/ 5 * 256);
            inds(inds<1) = 1;
            inds(inds>256) = 256;
            cmp = cmp(inds, :);
            scatter3(coors(:,1), coors(:,2), coors(:,3),makersize, cmp, 'filled')

        end
        frame = getframe(gcf);
        writeVideo(writerObj,frame);
        cla(gca)
    end
    close(writerObj);
    disp("finished.");
catch ME    
    close(writerObj);
    rethrow(ME)
end

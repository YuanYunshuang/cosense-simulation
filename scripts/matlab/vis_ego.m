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
limits = [-200, 0,-100, 100,0, 6];
makersize = 2;
filter = 1;
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
view(-30,30);  
hold on
writerObj = VideoWriter('j509e669_ego.avi');
writerObj.FrameRate = 10;
writerObj.Quality= 100;
open(writerObj);
 
%% Loop
try
    for i=1:(length(lidars_data))
        pcfile_ego = fullfile(lidars_data(i).folder, lidars_data(i).name);
        frame = lidars_data(i).name(1:end-4);
        metafile_ego = fullfile(lidars_data(i).folder, frame + "_meta.txt");
        ptCloud_ego = pcread(pcfile_ego);
        [eframe, etimestamp, etform] = read_meta(metafile_ego);
        if filter<1
            ptCloud_ego = pcdownsample(ptCloud_ego,'random',filter);
        end
        ptCloud_ego = pctransform(ptCloud_ego,etform);

        coors = ptCloud_ego.Location;
        labels = double(ptCloud_ego.Color) / 255;
        scatter3(coors(:,1), coors(:,2), coors(:,3),makersize, labels, 'filled')

        frame = getframe(gcf);
        writeVideo(writerObj,frame);
        cla(gca);
    end
    close(writerObj);
    disp("finished.");
catch ME
    close(writerObj);
end

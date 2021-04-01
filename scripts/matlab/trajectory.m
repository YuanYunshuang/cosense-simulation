clear all;
close all

%% Prepare data for visualization
% read trajectory data
data = load('trajectory_j509_e532.mat').data;
% vals = values(data);
% for id=1:data.Count
%     size(vals{id})
% end

%% visualize trajectory data
trajs = containers.Map;
ks = keys(data);
for key=ks
    coors = data(key{1});
    s = size(coors);
    for j=1:s(1)
        if ~isKey(trajs, string(coors(j, 1)))
            trajs(string(coors(j, 1))) = [[]];
        end
        traj = trajs(string(coors(j, 1)));
        traj(end + 1, :) = coors(j, :);
        trajs(string(coors(j, 1)))  = traj;
    end
end

fig = figure;
hold on 
keys = keys(trajs);
ids = [521; 529;532];
for c=1:trajs.Count
    if any(str2num(keys{c})==ids)
        keys{c}
        tj_m = trajs(keys{c});
        if str2num(keys{c})==532
            plot(tj_m(:,2), tj_m(:,3), 'r--o');
        else
            plot(tj_m(:,2), tj_m(:,3), '--*');
        end
        hold on;
    end
end


% Choose a sepcific number of nodes that are good geometrical distributed.

%% Generate a random network, all nodes lie in a 100x100 square
n_nodes = 20;
xs = [randi([-5, 5], 1,n_nodes / 2)*2 randi([-50, 50], 1,n_nodes / 2)];
ys = [randi([-50, 50], 1,n_nodes / 2) randi([-5, 5], 1,n_nodes / 2)*2];

center_node_idx = randi([0, n_nodes]);

%% Do sampling
n_samples = 10; % number of nodes that should be sampled
choosen_nodes = zeros(1,n_nodes);
% set center node as choosen nodes
choosen_nodes(center_node_idx) = 1;
for i=1:n_samples
    indices_choosen_nodes = find(choosen_nodes==1);
    indices_left_nodes = find(choosen_nodes==0);
    max_dist = 0;
    max_idx = -1;
    for j=1:length(indices_left_nodes)
        dists = (xs(indices_choosen_nodes) - xs(indices_left_nodes(j))).^2 + ...
                   (ys(indices_choosen_nodes) - ys(indices_left_nodes(j))).^2;
        [dist, I] = min(dists);
        if dist > max_dist
            max_dist = dist;
            max_idx = indices_left_nodes(j);
        end        
    end
    choosen_nodes(max_idx) = 1;
end

scatter(xs(find(choosen_nodes==1)), ys(find(choosen_nodes==1)),'g');
hold on;
scatter(xs(find(choosen_nodes==0)), ys(find(choosen_nodes==0)),'k');
scatter(xs(center_node_idx), ys(center_node_idx),'r');



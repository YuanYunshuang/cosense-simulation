import open3d as o3d

vis = o3d.visualization.Visualizer()
vis.create_window()

# geometry is the point cloud used in your animaiton
geometry = o3d.geometry.PointCloud()
vis.add_geometry(geometry)

for i in range(icp_iteration):
    # now modify the points of your geometry
    # you can use whatever method suits you best, this is just an example
    geometry.points = pcd_list[i].points
    vis.update_geometry(geometry)
    vis.poll_events()
    vis.update_renderer()
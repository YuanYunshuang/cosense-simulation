# Simulation for collective perception with CALAR & SUMO

# Run the simulation
## Generate routes and rerouting strategy 
- run python script `trafficgen.py` under directory _trafic_flow_ to regenerate routes if you made some modifications on road generation. Otherwise routes are already generated in folder _town5_.

## Activate Carla simulation server

## Cofigurate simulation parameters and start simulation
- config parameters are in _config.json_ file under dir Sumo, path for saving data is also defined there.
    - "root_path": where to save the generated data
    - "sensor_names":  which sensors you want to use for collecting data
- run the simulation under the project root dir
```bash
python main.py traffic_flow/town5/Town05.sumocfg --tls-manager carla --sumo-gui
```

# Simulation raw data 
## Data structure
- rootdir
    - vehicle_id...
        - camera
            - image...
            - image_meta...
        - camera_sem
            - images_sem_label...
            - image_sem_meta...
        - lidar
            - pointcloud...
            - pointcloud_meta...
        - lidar_sem
            - pointclouds_sem_label...
            - pointclouds_sem_meta...
    - info.csv

## Definition of file names and file contents
- __...__ means there are multiple such file or folders
- __rootdir__: __j__ xxx __e__ xxx states for junction, the junction number, ego and ego vehicle id respectively
- __vehicle_id__: vehicle_id
- __image, image_sem_label__: RGB image data in png format, file name = "<frame>.png", check carla documentation for color definition
- __image_meta, image_sem_meta__: meta information about the image with the same frame name, file name = "<frame>_meta.txt"
    - line1: frame, timestamp, fov, height, width
    - line2: roll, pitch, yaw, x, y, z
- __pointcloud, pointclouds_sem_label__: pointcloud raw data, file name = "<frame>.pcd", check carla documentation for color definition
- __pointcloud_meta, pointclouds_sem_meta__: meta information about the pointcloud with the same frame name, file name = "<frame>_meta.txt"
    - line1: frame, timestamp, horizontal_angle, n_channels
    - line2: roll, pitch, yaw, x, y, z
    - line3: 64 elements array, each element indicate the points number measured by the corresponding laser.

# Data formating 
- run the python script `formatting_data.py` in folder _scripts/python_ , before runing the script, you need to change the path(`in_path`) to the generated raw simulation data as well as the path(`out_path`) where you want to store the formatted data.
## Data structure
- the formatted data has 3 folders
    - cloud_ego: clouds collected by ego vehicle, no down-sampling
    - cloud_coop: point clouds of cooperative vehicles
    - cloud_fused: fused clouds(including ego-vehicle cloud), down-sampled through voxel grid(size=0.2m)
    - label_box: ground truth bounding boxes of all vehicles in the scene of each frame
- point cloud files:
    - all data are written in binary files. Data are all in `float32` length, 
    - each data point has 4 columns(x,y,z,label), following is the semantic meaning of labels
    ```python
        LABEL_COLORS = np.array([
        (0, 0, 0),       #0 Unlabeled
        (70, 70, 70),    #1 Buildings
        (100, 40, 40),   #2 Fences
        (55, 90, 80),    #3 Other
        (220, 20, 60),   #4 Pedestrians
        (153, 153, 153), #5 Poles
        (157, 234, 50),  #6 RoadLines
        (128, 64, 128),  #7 Roads
        (244, 35, 232),  #8 Sidewalks
        (107, 142, 35),  #9 Vegetation
        (0, 0, 142),     #10 Vehicles
        (102, 102, 156), #11 Walls
        (220, 220, 0),   #12 TrafficSign
        (70, 130, 180),  #13 Sky
        (81, 0, 81),     #14 Ground
        (150, 100, 100), #15 Bridge
        (230, 150, 140), #16 Railtrack
        (180, 165, 180), #17 GuardRail
        (250, 170, 30),  #18 TrafficLight
        (110, 190, 160), #19 Static
        (170, 120, 50),  #20 Dynamic
        (45, 60, 150),   #21 Water
        (145, 170, 100)  #22 Terrain
        ])
    ```

- filename: <junction_id>_<frame_id>.\<extention>
- label_box format: 
    - each row indicates one bounding box, 
    - column from left to right are: vehicle_id, vehicle_class, x, y, z, rx, ry, rz, l, w, h
    - x,y,z: the center point of the bounding box
    - rx, ry, rz: orientation
    - l, w, h:  vehicle size 














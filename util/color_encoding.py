import numpy as np

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
])  # normalize each channel to [0-1] if use Open3D Pointcloud
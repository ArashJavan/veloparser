### Veloparser

Veloparser is a simple application which does following:

- Supports just Velodyne VLP16 at the moment.
- Takes a pcap file recorded by Velodyne lidar as input.
- Extracts all Frames from the pcap file.
- Saves both data-frames and position-frames.
- __Data frames__ are saved as __Point Clouds (.pcd)__ and/or as __plain Text-File__. 
- __Position frames__ are saved only as __Text-File__
- Converts frame's timestamps to GPS Week of Second format for synchronization with IMU/GNSS devices
- Can be parameterizes by yaml file.

The reason why i wrote it, is simply that i could not find any simply way without installuing ROS (Robot operating software)
or other huge c++-based lib that does 'just' extract the point clouds and GPS-Timestamps from pcap-file.

##### Usage
python veloparser -p /home/user/my.pcap -o /home/user/output_folder -c params.yaml


##### Dependencies
Veloparser has follwoing package dependencies:
- dpkt
- numpy
- tqdm

Please make sure that all of those packages are installed (pip or conda).

##### Output
Below a sample out of 2 Points in a __point cloud file__

``
Time [musec], X [m], Y [m], Z [m], ID, Intensity, Latitude [Deg], Longitudes [Deg], Distance [m]
2795827803, 0.032293, 5.781942, -1.549291, 0, 6, 0.320, -15.000, 5.986
2795827806, 0.083565, 14.399564, 0.251350, 1, 6, 0.333, 1.000, 14.402
``

All __Point Cloud__ PCD-Files have follwoing fields:
1) X-Coordinate
2) Y-Coordinate
3) Z-Coordinate
4) Intensity

They can also be opened and visualized with any point-cloud rendering software like (open3d, pcl, ...)

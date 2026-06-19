'''
This program will be the main function that runs the dodge protocol

It should do the following:

- Subscribe to the LiDAR< IMU and wheel encoders of the TurtleBot4
    - Send all of that data to the mapper.py file
    - Send LiDAR data to the visual_odometry.py file
- Intake planned paths from the path_planner.py file
- Publish the planned path to the wheel encoders

All at every timestep (is this efficient?)
'''
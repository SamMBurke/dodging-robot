'''
This file will encompass the path planning for the dodging that the robot should do

- It will intake the LiDAR visual odometry data from the visual_odometry.py file
- It will then calculate the potential collision path
- It will then plan a dodging path if necessary
    - It will then send that dodging path to the dodge.py to publish the planned path
      to the wheel encoders
'''
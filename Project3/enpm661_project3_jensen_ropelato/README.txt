------------------------------------------------------------
  ENPM 661 - Spring 2020
  Project 3, Phase 3/4

  Nicholas Jensen, Rafael Ropelato
------------------------------------------------------------

Setup:
  - Move whole "enpm661_project3_jensen_ropelato"-folder to your /catkin_ws/src folder.
  - Build package ($ catkin_make  in /catkin_ws)

------------------------------------------------------------

Run:
  - Run the following line terminal:
    $ roslaunch enpm661_project3_jensen_ropelato enpm661_project3.launch arg1:=VALUE1 arg2:=VALUE2 ...
    (list of arguments see below)

  This launches Gazebo and starts the path-finding algorithm. (A Figure will open which shows the progress)
  When the goal is found, Close the Python Plot (Figure) which will start the publisher and run the robot in Gazebo.

------------------------------------------------------------

Known Issues:
  - Running the script (roslaunch enpm661enpm661_project3_jensen_ropelato ....) may lead to an
    error where Gazebo doesn't start correctly.
    FIX:  The error seems to only occur occasionally. Therefore, killing the command (CTRL+C) append
          re-running should fix the problem. (Repeat a few times)

------------------------------------------------------------
Arguments:

  xStart      - Start Point (X-Coordinate)                      default= -4.2
  yStart      - Start Point (Y-Coordinate)                      default= -3.2
  thetaStart  - Start Point (Theta Angle)(in RAD)               default=  0
  xGoal       - Goal Point (X-Coordinate)                       default=  0.0
  yGoal       - Goal Point (Y-Coordinate)                       default= -3.2
  rpm1        - RPM1 for ActionSet                              default=  90
  rpm2        - RPM2 for ActionSet                              default=  200
  clearance   - Clearance from Obstacles                        default=  0.3
  timestep    - Timestep for Path Calculation and Simulation    default=  1.0
  occupation_grid_size"   - Grid Size for Double Occupation     default=  0.1

  *Default values for for scenario of video 1.


#  Search and rescue project group 4
This is our amazing solution using 2 robots, with a slightly modified bug2 algorithm. The robots will, by default, drive to randomly generated points in their reserved region of the map. 
      It is a simple algorithm, as mentioned based on bug2 for the navigation, so as for 
the robots to reach their destinations. There is a running instance of 'bug2' per launched robots (of which there are 2). A separate controller is assigned the responsibility of processing the occupancy grid and coordination the navigator-controllers (of which there are one running instance per robot), by generating a new destinations as one is reached, based on the map. Similarly, a controller named (detectionFilter.py) separately does the job of receiving aruco detections, to conditionaly pass these on to the coordinator.

## How to run it:
First off all you need to navigate into the "search_rescue" folder in the project, followed by "colcon build", and "source install/setup.bash". After that is done you can start by launching a world: 
To run the project you first need to launch one of the worlds, robots and gazebo etc with one of the following commands: 

      - ros2 launch multi_robot_challenge_23 rescue_robots_w1.launch.py
      - ros2 launch multi_robot_challenge_23 rescue_robots_w2.launch.py
      - ros2 launch multi_robot_challenge_23 rescue_robots_w3.launch.py
      - ros2 launch multi_robot_challenge_23 rescue_robots_w4.launch.py
      - ros2 launch multi_robot_challenge_23 rescue_robots_w5.launch.py

After you have launched one of the worlds, gazebo etc you need to start the code for the robots, you will do that by running the command: 

      - ros2 launch sokogredd sokogredd.launch.py
      
You should now be done, and able to see the robots moving.

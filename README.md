
#  Search and rescue project group 4
This is our amazing solution using 2 robots, with a slightly modified bug2 algorithm paired with randomly made points around the map (similar to the PRM navigation algorithm). The robots will drive to the different points, 
spin around a 360, and if no aruco markers are found, drive to the next point until all the markers are found. 

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

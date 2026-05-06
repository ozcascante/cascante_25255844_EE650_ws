# Project Turtlebot3_house_navigation

## What I have done so far

(**Requires Navigation2**)

Started Project using a simpler example. PlanSys2_patrol_navigation_example:
- Copy this example as starting point.
- Change name to turtlebot3_house_navigation 
    - Modified: CMakeLists.txt & package.xml
    - Keep /pddl/patrol.pddl unchanged.
    - Modify launch/patrol_example_launch.py. comment out nav2_cmd as it was causing Rviz to show the house map and the turtleworld.
    - Modify src/ move_action_node.cpp to include each house room coordinate and added a room6.
    ```
        wp.pose.position.x = -2.4;
        wp.pose.position.y = 2.3;
        waypoints_["room1"] = wp;
        wp.pose.position.x = -6.2;
        wp.pose.position.y = 3.1;
        waypoints_["room2"] = wp;
        wp.pose.position.x = -6.3;
        wp.pose.position.y = -1.6;
        waypoints_["room3"] = wp;
        wp.pose.position.x = 4.8;
        wp.pose.position.y = 1.7;
        waypoints_["room4"] = wp;
        wp.pose.position.x = 6.2;
        wp.pose.position.y = -1.9;
        waypoints_["room5"] = wp;
        wp.pose.position.x = 1.2;
        wp.pose.position.y = 4.0;
        waypoints_["room6"] = wp;
    ```    

    - Laptop was struggling with the simulation, so I found changing some parameters in the param/nav2_params.yaml. Also decrease the Inflation Radius. (Robot struggling in corridor)
        - expected_planner_frequency: Change from 20.0 to  2.0. (The robot doesn't need to re-plan the long path every second).
        - controller_frequency: Change from 20.0 to 5.0. (This controls how often it calculates wheel speeds).
        - bt_loop_duration: increase it from 10 to  200.
        - Inflation Radius in costmap is too high, the robot thinks the doorway is blocked and give up. Lower inflation_radius to 0.2
- Created a commands.txt file. (made it executable). Commands to init the knowledge of the system and set the goal.Created a commands.txt file. (made it executable). Commands to init the knowledge of the system and set the goal.

```
set instance leia robot
set instance room1 waypoint
set instance room2 waypoint
set instance room3 waypoint
set instance room4 waypoint
set instance room5 waypoint
set instance room6 waypoint

set predicate (connected room1 room2)
set predicate (connected room2 room1)
set predicate (connected room2 room3)
set predicate (connected room3 room2)
set predicate (connected room1 room4)
set predicate (connected room4 room1)
set predicate (connected room4 room5)
set predicate (connected room5 room4)
set predicate (connected room4 room6)
set predicate (connected room6 room4)

set predicate (robot_at leia room3)
set goal (and (robot_at leia room6))
```

## How to run

In terminal 1:
```
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=-6.3 y_pose:=-1.6 use_sim_time:=True headless:=False  autostart:=True
```
In terminal 2:
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True headless:=False  autostart:=True map:=turtlebot3_house_navigation/maps/house_explored.yaml
```
In terminal 3:
```
ros2 launch turtlebot3_house_navigation patrol_example_launch.py use_sim_time:=True
```
In terminal 4:
```
ros2 run plansys2_terminal plansys2_terminal
>source commands.txt
> get plan
> run
```
```
rm -rf /tmp/plansys2_*
rm -fr /dev/shm/fastrtps_port*
```

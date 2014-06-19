Given some data of odometry of wheels and laser data this demo constructs using gmapping a map.

You can just launch:
```
roslaunch turtlebot_laser_work launch_all.launch
```

And it will launch all the necesarry stuff:

* wheels velocities publisher
* laser data publisher
* odom publisher from wheels info
* gmapping
* needed transforms (base_footprint to laser_link)
* rviz to watch what is happening


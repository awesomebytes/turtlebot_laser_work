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

The result looks like:

![odom and laser image](https://raw.githubusercontent.com/awesomebytes/turtlebot_laser_work/master/map_from_odom_and_laser.png)


Also you can try
```
roslaunch turtlebot_laser_work launch_laser_only.launch
```

To use laser_scan_matcher to publish odom frame. The results are a lot worse.

The result looks like:

![only laser image](https://raw.githubusercontent.com/awesomebytes/turtlebot_laser_work/master/map_from_only_laser.png)

# ros-streetwalk
> Final project of Robotics course. Work in progress...
> 
#### Commands
```sh
roslaunch ros-streetwalk run_single.launch name:=thymio10 world:=street_single
roslaunch ros-streetwalk task1.launch
rqt
```

```sh
roslaunch ros-streetwalk run.launch name:=thymio10 world:=street
roslaunch ros-streetwalk task2.launch robot_name:=thymio10
rqt
```

```sh
roslaunch ros-streetwalk run_multiple.launch name:=thymio10 world:=street
roslaunch ros-streetwalk task2.launch robot_name:=thymio10
roslaunch ros-streetwalk task2_2.launch robot_name:=thymio11
rqt
```

# Lane-following
> Final project of Robotics course. 
> 
> @ USI 20/21.
>

Github repository: https://github.com/okamiRvS/Lane-following

Video folder: https://drive.google.com/drive/folders/18hJCJqRr_7hdL7yg20SW_annQd0byjU1?usp=sharing

#### Prerequisites
The files in Gazebo11 have to be copied and merged to the Gazebo folder located e.g. at /usr/share/gazebo-11

#### Commands
Single Thymio:
```sh
roslaunch ros-streetwalk run.launch world:=street
roslaunch ros-streetwalk task2.launch
```

Multiple Thymio:
```sh
roslaunch ros-streetwalk run.launch world:=street
roslaunch ros-streetwalk task3.launch
```

If there are problems with the pitch camera, close the task.launch (not Gazebo) and try to run again this last script.
# siasun_ws

# 注意：
 - 新松目前试用的七轴协作机器人编码器方向设置不统一，在使用ROS前请逐个验证各轴方向是否一致，如果不一致可以在urdf文件内调整。
 - 需要自行安装moveit! ros-industrial相关内容

# 使用简介

## 启动moveit!界面

```
roslaunch srxr_moveit_config moveit_planning_execution.launch sim:=true
```

**sim:=true**打开的是仿真界面，可以执行moveit!各种应用的仿真，在连接实际机器人的时候需要去掉。

## 执行运动的示例程序

```
rosrun srxr_moveit_config srxr_move_group_interface_tutorial
```

<video id="video" controls="" preload="none" poster="http://om2bks7xs.bkt.clouddn.com/2017-08-26-Markdown-Advance-Video.jpg">
<source id="mp4" src="https://github.com/wxflamy/siasun_ws/blob/master/xmedia/Screencast2019-10.mp4" type="video/mp4">
</video>
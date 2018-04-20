# follower

1：首先启动turtlebot或虚拟仿真环境
2：修改server.py中socketIP为本机IP，启动server.py，然后在leader端启动client.py
3：可以启动rviz，通过tf和odom来观察leader和follower的运行情况
4：启动lead_follow_node节点，开始跟随，或者启动launch文件，这样就包含了速度平滑控制器，要修改对应的话题名字

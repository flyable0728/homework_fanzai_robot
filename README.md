# homework_fanzai_robot
1.程序位置
（1）核心程序为“capture_image.cpp”，位置在jaka_ws/src/jaka_driver/src/capture_image.cpp

（2）CmakeLists.txt和package.xml，因为电脑路径不同，请根据自己电脑对应路径更改里面的编译需求，缺少的包自行安装，注意更改的地方很多，请逐一对照。
更改好后放置于jaka_ws/src/jaka_driver/src

（3）jaka_ws/下应增加三个txt
“X-Auth-Token.txt”记录华为云api的X-Auth-Token
“api-web.txt”记录华为云api的调用网址
“calibration_data.txt”记录标定的转换矩阵，运行程序中进行标定后会自动生成。

（4）jaka_ws/下还会保存两张图片
“Objects-image.png “是拍摄需要抓取物块的图片，需要将这张图传给api获取像素位置
“Calibration-image.png “是标定时所需的图片

2.程序使用（catkin_make编译完成后）
（1）开启程序时请先在一个窗口运行如下代码来连接机器人
roslaunch jaka_driver robot_start_launch.launch ip:=10.5.5.100

（2）再开一个窗口，运行如下代码启动程序
rosrun jaka_driver capture_image 

（3）启动成功后会输出“Connected to robot at IP:[robot IP]”

（4）下一步跳出“Do you want to use the previous calibration result? (y/n):”
选择y使用上次标定数据，并跳到步骤（8）
选择n则进行标定

（5）标定开始，程序自动捕获当前照片，在照片上鼠标点击选择三个点，程序进行记录

（6）接下来移动机械臂，保持尾端Roll,Pitch,Yaw如下（可使用jaka的软件精确控制）
target_pose.rpy.rx = -3.14;          // Roll
    target_pose.rpy.ry = 0.0;          // Pitch
target_pose.rpy.rz = -3.14/ 2;       // Yaw
且保证尾部吸盘正好贴合到之前鼠标点击的点的位置，注意按照鼠标点击点的顺序进行移动，否则坐标变换会错误。每次向命令行输入y并回车才会记录机械臂数据。注意保证记录的时候其他人没有向机械臂发送service，否则无法记录。
推荐在捕获照片的时候在相应位置放上小物块，这样点击的时候可以点击小物块的中心，吸盘放置的时候也有参考。
程序默认设置为3个标定点，是最低要求且精度足够，如果精益求精可以更改程序中的标定次数，多标定几次。

（7）标定完毕，自动计算坐标转换矩阵并记录在jaka_ws/calibration_data.txt，输入y并回车进入下一步

（8）自动捕获当前图片，获得需要抓取的小物块像素位置并转换到机器人坐标系，准备开始运动

（9）输入y，回车。自动开启吸盘并移动到物块处，抓取并移动到指定放置位置。
再次输入y并回车，关闭吸盘，物块掉落。
（如果有更多物块）
再次输入y并回车，自动开启吸盘并移动到物块处，抓取并移动到指定放置位置。
如上重复，直至结束。

3.结果展示
见video.mp4









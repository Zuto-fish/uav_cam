# 图像接收与录制

1. 连接无人机
  
    在`rcvimg.cpp`中修改
    `const char* videoStreamUrl = "tcp://当前无人机IP地址:端口";`
 
2. 图像的无线接受与消息发布
    
    `rosrun uav_cam image_publisher  `

    发布话题为`/uav_image_raw`
3. 图像的消息订阅与显示

    `rosrun uav_cam image_subscriber`  
    
- 一起运行

    ```bash
    roslaunch uav_cam uav_image.launch   
    ```

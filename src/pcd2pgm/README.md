# pcd2pgm_package
点云pcd文件转二维栅格地图

完整程序代码: [Hinson-A/pcd2pgm_package: 点云pcd文件转二维栅格地图](https://github.com/Hinson-A/pcd2pgm_package)

## 运行方法：

下载编译后，

```shell
mkdir -p test_ws/src&& cd test_ws/src
git clone -b develop  https://github.com/Hinson-A/pcd2pgm_package
cd ../
catkin_make
```

编译完成后，查看 src/pcd2pgm_package/pcd2pgm/launch/中的run.launch文件，修改相应的文件路径及名称，launch文件如下：

```xml
<!-- -->
<launch>
<node pkg="pcd2pgm" name="pcd2pgm" type="pcd2pgm" output="screen">
<!-- 存放pcd文件的路径-->
<param name="file_directory" value= "/home/robot/map/" />
<!-- pcd文件名称-->
<param name="file_name" value= "map" />
<!-- 选取的范围　最小的高度-->
<param name="thre_z_min" value= "0.1" />
<!-- 选取的范围　最大的高度-->
<param name="thre_z_max" value= "1.5" />
<!--0 选取高度范围内的，１选取高度范围外的-->
<param name="flag_pass_through" value= "0" />
<!-- 半径滤波的半径-->
<param name="thre_radius" value= "0.5" />
<!-- 半径滤波的要求点数个数-->
<param name="thres_point_count" value= "10" />
<!-- 存储的栅格map的分辨率-->
<param name="map_resolution" value= "0.05" />
<!-- 转换后发布的二维地图的topic，默认使用map即可，可使用map_server保存-->
<param name="map_topic_name" value= "map" />
</node>

</launch>

```

修改完成后，运行程序

```shell
roslaunch run.launch
```

此时，通过map_server 保存该栅格地图

```shell
rosrun map_server map_saver
```


#include <pcl_viewer.h>

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;


void showPointCloud(PointCloudXYZI::Ptr cloud, std::string name, int R, int G, int B){
    pcl::PointXYZI point;
    point.x = 1.0f;
    point.y = 2.0f;
    point.z = 3.0f;
    point.intensity = 100.0f; // 强度值，对于XYZI点类型有效
    cloud->points.push_back(point);
 
    // 注意：在实际应用中，你需要添加更多的点到点云中
    // 这里只是为了演示如何创建和显示一个包含单个点的点云
 
    // 设置点云的大小（必须调用，否则点云可能无法正确显示）
    cloud->width = cloud->points.size();
    cloud->height = 1; // 表示这是一个无序点云
    cloud->is_dense = true; // 或者根据你的数据设置
 
    // 创建一个PCLVisualizer对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
    viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色为黑色
 
    // 创建一个点云颜色处理器
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_handler(cloud, R, G, B); // RGB颜色，白色
    viewer->addPointCloud<pcl::PointXYZI>(cloud, color_handler, name); // 添加点云到可视化器
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name); // 设置点的大小
 
    // 添加一个坐标系（可选）
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters(); // 初始化相机参数
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

 
    // 进入可视化循环
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100); // 100ms的延迟
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 或者使用PCL的sleep函数
    }
}

void showPointCloud(PointCloudXYZIN::Ptr cloud, std::string name, int R, int G, int B){
    pcl::PointXYZINormal point;
    point.x = 1.0f;
    point.y = 2.0f;
    point.z = 3.0f;
    point.intensity = 100.0f; // 强度值，对于XYZI点类型有效
    cloud->points.push_back(point);
 
    // 注意：在实际应用中，你需要添加更多的点到点云中
    // 这里只是为了演示如何创建和显示一个包含单个点的点云
 
    // 设置点云的大小（必须调用，否则点云可能无法正确显示）
    cloud->width = cloud->points.size();
    cloud->height = 1; // 表示这是一个无序点云
    cloud->is_dense = true; // 或者根据你的数据设置
 
    // 创建一个PCLVisualizer对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(name));
    viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色为黑色
 
    // 创建一个点云颜色处理器
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal> color_handler(cloud, R, G, B); // RGB颜色，白色
    viewer->addPointCloud<pcl::PointXYZINormal>(cloud, color_handler, name); // 添加点云到可视化器
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name); // 设置点的大小
 
    // 添加一个坐标系（可选）
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters(); // 初始化相机参数
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

 
    // 进入可视化循环
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100); // 100ms的延迟
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 或者使用PCL的sleep函数
    }
}






// // typedef pcl::PointXYZ PointT;
// // typedef pcl::PointCloud<PointT> PointCloudT;
// // PointCloudT = PointCloud<pcl::PointXYZ>

// // using PointType = pcl::PointXYZINormal;
// // using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
// // using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;
// // PointCloudXYZI::Ptr cloud_in_; // cloud_in_是当下雷达扫描到的点云
// // PointCloudXYZIN::Ptr rough_map_; // 原始地图点云粗处理点云
// // PointCloudXYZIN::Ptr refine_map_; // 原始地图点云精处理点云


// PointCloudXYZI::Ptr point_rough   PointCloudXYZI::Ptr cloud_in_ 
void pclViewer(PointCloudXYZIN::Ptr prior_point_cloud, PointCloudXYZIN::Ptr pointcloud_before_tf, PointCloudXYZIN::Ptr pointcloud_after_tf){
    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two vertically separated viewports
    // 创建两个垂直分离的视图
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal> cloud_in_color_h (prior_point_cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (prior_point_cloud, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (prior_point_cloud, cloud_in_color_h, "cloud_in_v2", v2);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal> cloud_tr_color_h (pointcloud_before_tf, 20, 180, 20);
    viewer.addPointCloud (pointcloud_before_tf, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal> cloud_icp_color_h (pointcloud_after_tf, 180, 20, 20);
    viewer.addPointCloud (pointcloud_after_tf, cloud_icp_color_h, "cloud_icp_v2", v2);

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    // std::stringstream ss;
    // ss << iterations;
    // std::string iterations_cnt = "ICP iterations = " + ss.str ();
    // viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    // viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100); // 100ms的延迟
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 或者使用PCL的sleep函数
    }

}
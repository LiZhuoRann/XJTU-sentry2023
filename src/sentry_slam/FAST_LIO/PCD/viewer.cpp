#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// 用于将参数传递给回调函数的结构体
struct CallbackArgs {
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pickPointCallback(const pcl::visualization::PointPickingEvent& event, void* args) {
    CallbackArgs* data = (CallbackArgs*)args;
    if (event.getPointIndex() == -1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);
    // 绘制红色点
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,
        "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int main() {
    std::string file_name("/home/xjturm/shaobing/src/sentry_slam/FAST_LIO/PCD/rainbow.pcd");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    // 加载点云
    if (pcl::io::loadPCDFile(file_name, *cloud) == -1) {
        std::cerr << "could not load file: " << file_name << std::endl;
    }
    std::cout << cloud->points.size() << std::endl;
    // 显示点云
    viewer->addPointCloud(cloud, "cloud");
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    // 添加点拾取回调函数
    CallbackArgs  cb_args;
    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(pickPointCallback, (void*)&cb_args);
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

    viewer->spin();
    std::cout << "done." << std::endl;

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    return 0;
}

#include <iostream>
#include <pcl-1.10/pcl/io/pcd_io.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.10/pcl/common/common_headers.h>


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


int main(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/bhavik/projects/compVision/obstacle-detection/data/simpleHighway.pcd", *cloud);

    std::cout <<cloud->width * cloud -> height << " Data points from Simple Highway pcd file"<< std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer;

    viewer = simpleVis(cloud);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
    
    return 0;
}


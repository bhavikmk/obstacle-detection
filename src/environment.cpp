/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor in Heap memory using new keyword
    Lidar *lidar = new Lidar(cars, 0.00);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer,lidar->position, inputCloud);
    renderPointCloud(viewer,inputCloud, "inputCloud"); 
    // TODO:: Create point processor
    
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; //On Stack
    
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../data/pcd/data_2/0000000000.pcd");

    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../data/pcd/data_2");

    // create streamer 
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;    

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    while (!viewer->wasStopped ())
    {
        // remove all point clouds
        
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load and run obstacle detection process

        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        // cityBlock(viewer, pointProcessorI, inputCloud);
        
        // filter input cloud 
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = pointProcessorI->FilterCloud(inputCloudI, 0.3, Eigen::Vector4f (-20, -6, -2, 1), Eigen::Vector4f ( 30, 7, 5, 1));
        
        // segment cloud
        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(cloudFiltered, 50, 0.3);
        
        // apply clustering in segmented cloud
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(segmentCloud.second, 0.3, 10, 1000);
        
        // render clouds
        renderPointCloud(viewer,inputCloudI, "inputCloud");


	    int clusterId = 0;
	    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
        // render bounding box on cloud        
        for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
        {
            // std::cout << "cluster size ";
            // pointProcessorI->numPoints(cluster);
            // renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);

            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);

            ++clusterId;
        }
        
        // Increment to next scene and if the last scene is reached loop back to first scene
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();

    } 
}
/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
#include <pcl/common/pca.h>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
//#include "quiz/cluster/kdtree.h"


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
    enum renderOption {Rays, PointCloud, Clustering};

    renderOption rOption = Clustering;

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    switch(rOption)
    {
        case Rays :
        {
            // RENDER OPTIONS
            bool renderScene = true;
            std::vector<Car> cars = initHighway(renderScene, viewer);
            // TODO:: Create lidar sensor
            Lidar* lidar = new Lidar(cars, 0);
            //pcl::PointCloud<pcl::PointXYZ>::Ptr 
            inputCloud = lidar->scan(); 
            renderRays(viewer, lidar->position, inputCloud);
            break;
        }            
            
        case PointCloud :
        {
            // RENDER OPTIONS
            bool renderScene = false;
            std::vector<Car> cars = initHighway(renderScene, viewer);        
            // TODO:: Create lidar sensor
            Lidar* lidar = new Lidar(cars, 0);
            //pcl::PointCloud<pcl::PointXYZ>::Ptr 
            inputCloud = lidar->scan();
            renderPointCloud(viewer, inputCloud, "Input Point Cloud"); 
            // TODO:: Create point processor
            ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
            ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

            //std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../../src/sensors/data/pcd/simpleHighway.pcd");
            //auto streamIterator = stream.begin();

            //pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;
            //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

            bool customRANSAC = true;

            if(customRANSAC)
            {
                std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->CustomRANSAC(inputCloud, 500, 0.2);
                renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
                renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
            }
            else
            {
                std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 500, 0.2);
                renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
                renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
            }
            break;
        }
        case Clustering:
        {
            // RENDER OPTIONS
            bool renderScene = false;
            std::vector<Car> cars = initHighway(renderScene, viewer);        
            // TODO:: Create lidar sensor
            Lidar* lidar = new Lidar(cars, 0);
            //pcl::PointCloud<pcl::PointXYZ>::Ptr 
            inputCloud = lidar->scan();
            //renderPointCloud(viewer, inputCloud, "Input Point Cloud"); 
            // TODO:: Create point processor
            ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
            ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

            std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->CustomRANSAC(inputCloud, 500, 0.2);
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

            int clusterId = 0;
            std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

            bool render_cluster = true;
            bool render_box = true;

            for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
            {
                if(render_cluster)
                {
                    std::cout << "cluster size:";
                    pointProcessor->numPoints(cluster);
                    renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId % 3]);
                }
                if(render_box)
                {
                    //Box box = pointProcessor->BoundingBox(cluster);
                    BoxQ box = pointProcessor->PCABoundingBox(cluster);
                    renderBox(viewer, box, clusterId);
                }
                clusterId++;
            }
            renderPointCloud(viewer, segmentCloud.second, "planeCloud");
            break;
        }
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ---------------------------------------------------- 
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;

    enum renderOption {Acquired, Filtered, Segmented};
    renderOption rOption = Segmented;

    switch (rOption)
    {
        case Acquired:
        {
            renderPointCloud(viewer,inputCloud,"inputCloud");
            break;
        }

        case Filtered:
        {
            filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f (-1000, -1000, -30, 1), Eigen::Vector4f (1000, 1000, 1000, 1));
            renderPointCloud(viewer,filterCloud,"filterCloud");
            break;
        }

        case Segmented:
        {
            filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f (-10, -6, -3, 1), Eigen::Vector4f (50, 7, 1000, 1));
            std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->CustomRANSAC(filterCloud, 500, 0.2); // Slower with CustomRANSAC !!!
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 20, 1500);

            int clusterId = 0;
            std::vector<Color> cluster_color = {Color(1,1,0), Color(1,0,0), Color(0,0,1)};

            bool render_cluster = true;
            bool render_box = true;

            for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
            {
                if(render_cluster)
                {
                    std::cout << "cluster size:";
                    pointProcessorI->numPoints(cluster);
                    renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), cluster_color[clusterId % 3]);
                }
                if(render_box)
                {  
                    Box box = pointProcessorI->BoundingBox(cluster);
                    //BoxQ box = pointProcessorI->PCABoundingBox(cluster);
                    renderBox(viewer, box, clusterId);
                }
                clusterId++;
            }
            renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
            break;
        }
    }
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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    enum cloudType {simpleCloud, realCloud};
    cloudType cloud = realCloud;
    switch (cloud)
    {
        case simpleCloud:
        {
            simpleHighway(viewer);
            
            while (!viewer->wasStopped ())
            {
                viewer->spinOnce ();
            }
            break;
        }
        case realCloud:
        {
            ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
            std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1", argv);
            auto streamIterator = stream.begin();
            pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
            //cityBlock(viewer, pointProcessorI, inputCloudI); //CustomRANSAC: Floating point exception (core dumped) due to empty point cloud
            
            while (!viewer->wasStopped ())
            {
                // Clear viewer
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();
                // Load pcd and run obstacle detection process
                inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
                cityBlock(viewer, pointProcessorI, inputCloudI);
                streamIterator++;
                if(streamIterator == stream.end())
                    streamIterator = stream.begin();

                viewer->spinOnce ();
            }
            break;
        }
    }    
    return 0;
}
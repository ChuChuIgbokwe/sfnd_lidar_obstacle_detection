/* \author Aaron Brown */
// Create simple 3d highway environment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene) {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false; // This renders the scene: road, cars, etc
    std::vector<Car> cars = initHighway(renderScene, viewer);
    bool render_obstacles = true;
    bool render_plane = true;
    bool render_clusters = true;
    bool render_box = true;

    //Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);
    //Generate an input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //This allows us to observe the rays in the simulator
    //renderRays(viewer, lidar->position , inputCloud);
    // This allows us to view the point cloud created by the lidar
    //renderPointCloud(viewer, inputCloud, "inputCloud", Color(1, 1, 1));

    //Create point processor
    //ProcessPointClouds<pcl::PointXYZ> pointProcessor; //stack
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); //heap
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
            pointProcessor->SegmentPlaneRANSAC(inputCloud, 100, 0.2);

    // Render point cloud segments in different colours
    if (render_obstacles) {
        renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1, 0, 0));
    }
    if (render_plane) {
        renderPointCloud(viewer, segmentCloud.second, "roadCloud", Color(0, 1, 0));
    }

    // cluster the outliers/obstacles from the segmented point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(
            segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        if (render_clusters) {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        }
        if (render_box) {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd(
            "../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, inputCloud, "inputCloud");
    // Experiment with the ? values and find what works best
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2,
                                               Eigen::Vector4f(-30, -5, -2, 1),
                                               Eigen::Vector4f(30, 7, .4, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlaneRANSAC(
            filterCloud, 100, 0.2);
//    renderPointCloud(viewer, filterCloud, "filterCloud");
    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "roadCloud", Color(0, 1, 0));


}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle) {
        case XY :
            viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
            break;
        case TopDown :
            viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
            break;
        case Side :
            viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
            break;
        case FPS :
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}


int main(int argc, char **argv) {
    std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
//    simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}
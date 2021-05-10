/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	// For max iterations
    while (maxIterations--) {
        // Randomly sample subset and fit line
        // This ensures that the two points picked are unique
        std::unordered_set<int> inliers;
        while (inliers.size() <2){
            inliers.insert(rand() % cloud->points.size());
        }

        auto point_sampled = inliers.begin();
        float x1 = cloud->points[*point_sampled].x;
        float y1 = cloud->points[*point_sampled].y;
        point_sampled++;
        float x2 = cloud->points[*point_sampled].x;
        float y2 = cloud->points[*point_sampled].y;

        /*
        Equation of a Line Through Two Points in 2D
        For variables x and y and coefficients a, b, and c, the general equation of a line is:
        Ax + By + c = 0Ax+By+c=0

        Given two points: point1 (x1, y1) and point2 (x2, y2), the line through point1 and point2 has the specific form:
        (y1 -y2)x + (x2 -x1)y + (x1*y2 -x2*y1) = 0
         */
        float a = (y1 - y2);
        float b = (x2 - x1);
        float c = (x1 * y2 - x2 * y1);

        for (auto index = 0; index < cloud->points.size(); index++){
            if(inliers.count(index) > 0){ //if point chosen is already on the line, don't do anything
                continue;
            }
            pcl::PointXYZ point = cloud->points[index];
            float x3 = point.x;
            float y3 = point.y;
            // Measure distance between every point and fitted line
            float d = fabs(a * x3 + b * y3 + c) / sqrt(a*a + b*b);
            // If distance is smaller than threshold count it as inlier
            if (d < distanceTol){
                inliers.insert(index);
            }
        }
        // Return indices of inliers from fitted line with most inliers
       if (inliers.size() > inliersResult.size()){
           inliersResult = inliers;
       }
    }

	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    while (maxIterations--) {
        // Randomly sample subset and fit line
        // This ensures that the three points picked are unique
        std::unordered_set<int> inliers;
        while (inliers.size() <3){
            inliers.insert(rand() % cloud->points.size());
        }

        auto point_sampled = inliers.begin();
        float x1 = cloud->points[*point_sampled].x;
        float y1 = cloud->points[*point_sampled].y;
        float z1 = cloud->points[*point_sampled].z;
        point_sampled++;
        float x2 = cloud->points[*point_sampled].x;
        float y2 = cloud->points[*point_sampled].y;
        float z2 = cloud->points[*point_sampled].z;
        point_sampled++;
        float x3 = cloud->points[*point_sampled].x;
        float y3 = cloud->points[*point_sampled].y;
        float z3 = cloud->points[*point_sampled].z;

        /*
        Equation of a Plane through Three Points
        Ax + By + Cz + D = 0
        For
        point1 = (x1, y1, z1)
        point2 = (x2, y2, z2)
        point3 = (x3, y3, z3)
        Use point1point1 as a reference and define two vectors on the plane v1and v2 as follows:

        Vector v1v1 travels from point1 to point2.
        Vector v2v2 travels from point1 to point3
        v1=<x2−x1,y2−y1,z2−z1>
        v2=<x3−x1,y3−y1,z3−z1>

        Find normal vector to the plane by taking cross product of v1 \times v2v1×v2:
        v1×v2=<(y2−y1)(z3−z1)−(z2−z1)(y3−y1),
        (z2-z1)(x3-x1)-(x2-x1)(z3-z1),
        (x2-x1)(y3-y1)-(y2-y1)(x3-x1)>

        To simplify notation we can write it in the form v1×v2=<i,j,k>
         then ,

        i(x-x1)+j(y-y1)+k(z-z1) = 0,
        ix + jy + kz -( ix1 + jy1 + kz1 ) = 0
        A = i
        B = j
        C = k
        D=−(ix1+jy1+kz1)
         */
        float a, b, c, d;
        a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        d = -1 * (a * x1 + b * y1 + c * z1);


        for (auto index = 0; index < cloud->points.size(); index++){
            if(inliers.count(index) > 0){ //if point chosen is already on the line, don't do anything
                continue;
            }
            pcl::PointXYZ point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;
            // Measure distance between every point and fitted line
            float distance = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a*a + b*b + c*c);
            // If distance is smaller than threshold count it as inlier
            if (distance < distanceTol){
                inliers.insert(index);
            }
        }
        // Return indices of inliers from fitted line with most inliers
        if (inliers.size() > inliersResult.size()){
            inliersResult = inliers;
        }
    }

    return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}

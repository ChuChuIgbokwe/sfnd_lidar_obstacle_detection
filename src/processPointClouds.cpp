// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
    std::cout << cloud->points.size() << std::endl;
}

/**
 * @brief This function filters a pointcloud using a voxel grid. Afterwhich the lidar points on the roof of the car
 * are extracted and removed
 * @tparam PointT
 * @param cloud: cloud to be filtered
 * @param filterRes
 * @param minPoint: describes min point of region of interest
 * @param maxPoint: describes max point of region of interest
 * @return filtered/downsampled point cloud
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
                                        Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Fill in the function to do voxel grid point reduction and region based filtering
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloudROI(new pcl::PointCloud<PointT>);

    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*filteredCloud);

    std::cerr << "PointCloud after filtering: " << filteredCloud->width * filteredCloud->height
              << " data points (" << pcl::getFieldsList(*filteredCloud) << ")." << std::endl;

    pcl::CropBox<PointT> regionOfInterest(true);
    regionOfInterest.setMin(minPoint);
    regionOfInterest.setMax(maxPoint);
    regionOfInterest.setInputCloud(filteredCloud);
    regionOfInterest.filter(*cloudROI); // save results to cloudROI

    // remove the point cloud roof points (where lidar hits the roof of the car) from the cloud ROI
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudROI);
    roof.filter(indices); // get the indices of the cloud points that fit within the roof box

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (auto point: indices) {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud(cloudROI);
    extract.setIndices(inliers);
    extract.setNegative(true); // Removes points outside the range of the argument of extract.setIndices()
    extract.filter(*cloudROI); // all the outliers are assigned to the obstacle cloud

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudROI;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,
                                           typename pcl::PointCloud<PointT>::Ptr cloud) {

    //Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr roadCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // populate the roadCloud pointcloud with the inliers
    for (auto index: inliers->indices) {
        roadCloud->points.push_back(cloud->points[index]);
    }
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    // inverted behaviour.  Removes points outside the range of the argument of extract.setIndices()
    extract.setNegative(true);
    extract.filter(*obstacleCloud); // all the outliers are assigned to the obstacle cloud

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentationResult
            (obstacleCloud, roadCloud);

    return segmentationResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                         float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    /* Fill in this function to find inliers for the cloud and to segment the cloud into the driveable plane and
    obstacles */

    pcl::SACSegmentation<pcl::PointXYZ> seg;  // Create the segmentation object
    //inliers have  header and a vector. The vector contains the indices of the of the inlier points  in the pointcloud
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    // Check if inliers vector is empty
    if (inliers->indices.size() == 0) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(
            inliers, cloud);
    return segResult;
}
/**
 * @brief
 * @tparam PointT
 * @param cloud
 * @param maxIterations
 * @param distanceThreshold
 * @return
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlaneRANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
                                               float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    while (maxIterations--) {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers; // This ensures that the three points picked are unique
        while (inliers.size() < 3) {
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

        for (auto index = 0; index < cloud->points.size(); index++) {
            if (inliers.count(index) > 0) { //if point chosen is already on the line, don't do anything
                continue;
            }
            PointT point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;
            // Measure distance between every point and fitted line
            float distance = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);
            // If distance is smaller than threshold count it as inlier
            if (distance < distanceThreshold) {
                inliers.insert(index);
            }
        }
        // Return indices of inliers from fitted line with most inliers
        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }
    if (inliersResult.size() == 0) {
        std::cerr << "No inliers found." << std::endl;
    }
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++) {
        PointT point = cloud->points[index];
        if (inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,
                                                                                                      cloudInliers);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize,
                                       int maxSize) {

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    // Loop through point cloud clusters and add them to the clusters vector
    for (pcl::PointIndices getIndices: clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for (int index: getIndices.indices) {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
              << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                               boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


/**
 * @brief This is a helper function that recursively checks if points in the kd tree belong to the same cluster
 * @param index: index of points in the processed vector
 * @param points: points in the kd tree
 * @param cluster: A vector that keeps track of indices of processed points
 * @param processed: a vector that keeps track of which point has been processed
 * @param tree:  Kd Tree object
 * @param distanceTol: distance limit for classifying points as part of the same cluster
 */
template<typename PointT>
void
ProcessPointClouds<PointT>::proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster,
          std::vector<bool>& processed, KdTree *tree, float distanceTol)
{
    processed[index] = true;
    cluster.push_back(index);

    std::vector<int> nearestPointId = tree->search(cloud->points[index], distanceTol);

    for( auto id: nearestPointId)
    {
        if (!processed[id]){
            proximity(id, cloud, cluster, processed, tree, distanceTol);
        }
    }
}

/**
 * @brief: a function return list of indices for each cluster
 * @param points: points in the kd tree
 * @param tree: kdtree object
 * @param distanceTol:  distance limit for classifying points as part of the same cluster
 * @return vector of ints
 */
template<typename PointT>
std::vector<std::vector<int>>
ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree, float distanceTol) {

    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(cloud->points.size(), false);
    int i = 0;
    while(i < cloud->points.size())
    {
        if (processed[i])
        {
            i++;
            continue;
        }
        // If point has not been processed Create cluster
        std::vector<int> cluster;
        proximity(i, cloud, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
        i++;
    }
    return clusters;

}
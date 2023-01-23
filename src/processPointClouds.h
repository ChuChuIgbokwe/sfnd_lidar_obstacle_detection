// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <boost/filesystem.hpp>

/**
 * @brief: Structure to represent node of kd tree
 */
struct Node {
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId)
            : point(arr), id(setId), left(NULL), right(NULL) {}
};

/**
 * @brief A kd tree initialized with a null root
 */
template<typename PointT>
struct KdTree {
    Node *root;

    KdTree()
            : root(NULL) {}
    /**
     * @brief This function is a helper function that recursively inserts points into the 2D kd tree
     * @param depth: depth of the tree. Used to determine whether to compare x or y coordinates before insertion
     * @param point: vector containing the x and y coordinates of point being inserted
     * @param id: id of the point being inserted
     */
    void insertHelper(Node *&node, uint depth, std::vector<float> point, int id) {
        //Check if tree is empty
        if (node == NULL) {
            node = new Node(point, id);
        }
        else {
            uint currentDepth = depth % 2;
            if (point[currentDepth] < (node->point[currentDepth])) {
                insertHelper(node->left, depth + 1, point, id);
            } else {
                insertHelper(node->right, depth + 1, point, id);
            }
        }
    }

    /**
     * @brief this function inserts a new point into the tree. It creates a new node and place correctly with in the root
     * @param point: point being inserted into the tree
     * @param id: point id
     */
    void insert(std::vector<float> point, int id) {
        insertHelper(*&root, 0, point, id);
    }

    /**
     * @brief This function is a helper function that recursively search for a point in the 2D kd tree
     * @param target: point being searched for
     * @param node: node object to be se
     * @param depth: depth of the tree. Used to determine whether to compare x or y coordinates before insertion
     * @param distanceTol: Distance tolerance b
     * @param ids: point id's
     */
    void searchHelper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids) {
        if (node != NULL) {
            // First check if a point is within the distance tolerance +/- range
            if ((node->point[0] >= target[0] - distanceTol) and (node->point[0] <= target[0] + distanceTol) and
                (node->point[1] >= target[1] - distanceTol) and (node->point[1] <= target[1] + distanceTol))
            {
                float distance = sqrt(pow((node->point[0] - target[0]), 2) + pow((node->point[1] - target[1]), 2));
                if (distance <= distanceTol) {
                    ids.push_back((node->id));
                }
            }

            // check across boundary
            // using the depth % 2 logic allows you to alternate between x and y comparisons
            if ((target[depth % 2] - distanceTol) < node->point[depth % 2]) {
                searchHelper(target, node->left, depth + 1, distanceTol, ids);
            }
            if ((target[depth % 2] + distanceTol) > node->point[depth % 2]) {
                searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
        }
    }

    /**
     * @brief: This function returns a list of point ids in the tree that are within distance of target
     * @param target
     * @param distanceTol
     * @return
     */
    std::vector<int> search(std::vector<float> target, float distanceTol) {

        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }
};

class path;

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlaneRANSAC(typename
    pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    void proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster,
              std::vector<bool>& processed, typename KdTree<PointT>::KdTree* tree, float distanceTol);

    std::vector<std::vector<int>>euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, typename
    KdTree<PointT>::KdTree* tree, float distanceTol);
};
#endif /* PROCESSPOINTCLOUDS_H_ */
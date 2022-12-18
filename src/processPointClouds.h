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
#include "custom/cluster/kdtree.h"
#include <unordered_set>


template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> CustomSegmentPlane(const typename pcl::PointCloud<PointT>::Ptr &cloud, int maxIterations, float distanceThreshold);

    std::unordered_set<int> CustomRansac(const typename pcl::PointCloud<PointT>::Ptr &cloud, int maxIterations, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> CustomClustering(const typename pcl::PointCloud<PointT>::Ptr &cloud, float clusterTolerance, int minSize, int maxSize);

    std::vector<std::vector<int>> CustomEuclideanCluster(const std::vector<std::vector<float>> &points, KdTree *&tree, float distanceTol, float minSize, float maxSize);
    
    void CustomEuclideanClusterHelper(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, int index, std::vector<int> &cluster, std::vector<bool> &processed);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */
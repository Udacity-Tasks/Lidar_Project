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
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomSegmentPlane(const typename pcl::PointCloud<PointT>::Ptr &cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliers = CustomRansac(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    if (cloudInliers->points.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        exit(-1);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;
}

// Custom Ransac
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::CustomRansac(const typename pcl::PointCloud<PointT>::Ptr &cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations 
    for (size_t i = 0; i < maxIterations; i++)
    {
        std::unordered_set<int> tempInliers;
        while (tempInliers.size() < 3)
        {
            tempInliers.insert((rand() % cloud->points.size()));
        }

        
        auto itr = tempInliers.begin();
        PointT randomP1 = cloud->points[*itr];
        itr++;
        PointT randomP2 = cloud->points[*itr];
        itr++;
        PointT randomP3 = cloud->points[*itr];

        float planeA = ((randomP2.y - randomP1.y) * (randomP3.z - randomP1.z)) - ((randomP2.z - randomP1.z) * (randomP3.y - randomP1.y));
        float planeB = ((randomP2.z - randomP1.z) * (randomP3.x - randomP1.x)) - ((randomP2.x - randomP1.x) * (randomP3.z - randomP1.z));
        float planeC = ((randomP2.x - randomP1.x) * (randomP3.y - randomP1.y)) - ((randomP2.y - randomP1.y) * (randomP3.x - randomP1.x));
        float planeD = -((planeA * randomP1.x) + (planeB * randomP1.y) + (planeC * randomP1.z));

        for (size_t index = 0; index < cloud->points.size(); index++)
        {
            if (tempInliers.count(index) > 0)
                continue;

            PointT point = cloud->points[index];
            double distance = fabs((planeA * point.x) + (planeB * point.y) + (planeC * point.z) + planeD) / (sqrt((planeA * planeA) + (planeB * planeB) + (planeC * planeC)));

            if (distance <= distanceTol)
                tempInliers.insert(index);
        }

        if (tempInliers.size() > inliersResult.size())
        {
            inliersResult = tempInliers;
        }

    }

    return inliersResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomClustering(const typename pcl::PointCloud<PointT>::Ptr &cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    // Create a KD-Tree object for the search method of the extraction
    KdTree *tree3D = new KdTree;
    // create a 2D vector naming pointsPool to store the points
    std::vector<std::vector<float>> pointsPool(cloud->points.size());
    
    for (int i=0; i<cloud->points.size(); i++)
    {   
        // Store the points in the pointsPool as x,y,z
        std::vector<float> pointInPool = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}; 
        // Insert the points into the tree
        tree3D->insert(pointInPool, i);
        // Insert the pointInPool into the pointsPool
        pointsPool[i] = pointInPool;
    }

    // Create a vector of vectors to store the indices of the clusters
    std::vector<std::vector<int>> clustersIndices = CustomEuclideanCluster(pointsPool, tree3D, clusterTolerance, minSize, maxSize);
    
    // Iterate through the clustersIndices and push the points into the clusters
    for (std::vector<int> getIndices: clustersIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for (int index : getIndices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// Define a function to perform the euclidean clustering
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::CustomEuclideanCluster(const std::vector<std::vector<float>> &points, KdTree *&tree, float distanceTol, float minSize, float maxSize)
{
    // Create a vector of vectors to store the indices of the clusters
    std::vector<std::vector<int>> clusters;
    // Create a vector to store the indices of the points that have been processed
    std::vector<bool> processed(points.size(), false);

    // Iterate through the points
    for (size_t i=0; i<points.size(); i++)
    {
        // If the point has been processed, skip it
        if (!processed[i])
        {
        // Create a vector to store the indices of the points in the cluster
            std::vector<int> cluster;
                
            // Call the proximity function to find the indices of the points in the cluster
            CustomEuclideanClusterHelper(points, tree, distanceTol, i, cluster, processed);
            // If the size of the cluster is between the minSize and maxSize, push the cluster into the clusters vector
            if (cluster.size() >= minSize && cluster.size() <= maxSize)
                clusters.push_back(cluster);
        }
    }
    
    return clusters;
}

template <typename PointT>
void ProcessPointClouds<PointT>::CustomEuclideanClusterHelper(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, int index, std::vector<int> &cluster, std::vector<bool> &processed)
{
    if (!processed[index])
    {
        // Mark the point as processed
        processed[index] = true;
        // Push the index of the point into the cluster
        cluster.push_back(index);
        // Find the indices of the points in the proximity of the point
        std::vector<int> nearbyPoints = tree->search(points[index], distanceTol);
        // Iterate through the nearby points
        for (int id : nearbyPoints)
        {
            // If the point has not been processed, call the proximity function to find the indices of the points in the cluster
            if (!processed[id])
            {
                CustomEuclideanClusterHelper(points, tree, distanceTol, id, cluster, processed);
            }
        }
    }
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>());

    pcl::CropBox<PointT> region(true); // Create the region filtering object
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud);
    region.filter(*cloudFiltered);

    pcl::VoxelGrid<PointT> vg; // Create the Voxel Grid filtering object
    vg.setInputCloud(cloudFiltered);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    //typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>());

    // Roof filtering
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true); 
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudFiltered);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudFiltered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudFiltered);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;


    return cloudFiltered;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    // Create filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);


    return segResult;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

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
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
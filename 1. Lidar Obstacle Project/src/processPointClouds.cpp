#include "processPointClouds.h"



// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {
}



// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {
}



template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
 std::cout << cloud->points.size() << std::endl;
}



template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) 
{
    // Time segmentation process
    const auto startTime{ std::chrono::steady_clock::now() };

	pcl::VoxelGrid<PointT> voxelGrid;
 	voxelGrid.setInputCloud (cloud);
    voxelGrid.setLeafSize (filterRes, filterRes, filterRes);
  
    typename pcl::PointCloud<PointT>::Ptr filteredVoxelCloud (new pcl::PointCloud<PointT>);
    voxelGrid.filter (*filteredVoxelCloud);
  

    pcl::CropBox<PointT> cropBox(true);
    cropBox.setInputCloud(filteredVoxelCloud);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);

	typename pcl::PointCloud<PointT>::Ptr croppedCloud (new pcl::PointCloud<PointT>);
 	cropBox.filter(*croppedCloud);

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.0));
    roof.setInputCloud(croppedCloud);

    std::vector<int> inlierIndices;
    roof.filter(inlierIndices);

  
    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
    for (int index : inlierIndices) 
    { 
      inliers->indices.push_back(index); 
    }

    pcl::ExtractIndices<PointT> extractedIndices;
    extractedIndices.setInputCloud(croppedCloud);
    extractedIndices.setIndices(inliers);
    extractedIndices.setNegative(true);
    extractedIndices.filter(*croppedCloud);


    const auto endTime{ std::chrono::steady_clock::now() };
    const auto elapsedTime{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime) };
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return croppedCloud;
}



template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,typename pcl::PointCloud<PointT>::Ptr cloud) 
{

    // TODO: Create two new point clouds, one cloud with obstacles and other
    // with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstacleCloud{ new pcl::PointCloud<PointT> };
    typename pcl::PointCloud<PointT>::Ptr roadCloud{ new pcl::PointCloud<PointT> };


    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(false);
    extract.filter(*roadCloud);

    extract.setNegative(true);
    extract.filter(*obstacleCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, roadCloud);

    return segResult;
}



template <typename PointT> std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {

    // Time segmentation process
    auto startTime{ std::chrono::steady_clock::now() };
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices() };
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::SACSegmentation<PointT> segmentation;



    segmentation.setOptimizeCoefficients(true);
    segmentation.setMaxIterations(maxIterations);
    segmentation.setDistanceThreshold(distanceThreshold);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setInputCloud(cloud);
    segmentation.segment(*inliers, *coefficients);

    if (cloud->points.empty()) { std::cout << "Failed to segment points." << std::endl; }

    auto endTime{ std::chrono::steady_clock::now() };
    auto elapsedTime{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime) };
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult{SeparateClouds(inliers, cloud)
    };
    return segResult;
}



// START OF MY ALGORITHM
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, const float distanceTol) 
{
      // Time segmentation process
   auto startTime = std::chrono::steady_clock::now();
   std::unordered_set<int> inliersResult;
   while(maxIterations--)
   {
     //pick three points
     std::unordered_set<int> inliers;
     while (inliers.size() < 3)
     {
       inliers.insert(rand()%(cloud->points.size()));
     }

     float x1, y1, z1, x2, y2 , z2, x3, y3, z3;
     auto itr = inliers.begin();
     x1 = cloud->points[*itr].x;
     y1 = cloud->points[*itr].y;
     z1 = cloud->points[*itr].z;
     itr++;
     x2 = cloud->points[*itr].x;
     y2 = cloud->points[*itr].y;
     z2 = cloud->points[*itr].z;
     itr++;
     x3 = cloud->points[*itr].x;
     y3 = cloud->points[*itr].y;
     z3 = cloud->points[*itr].z;

     // create vector with resepct to Point1
     // crossproduct
     float A = (y2-y1)*(z3-z1) -(z2-z1)*(y3-y1);
     float B = (z2-z1)*(x3-x1) -(x2-x1)*(z3-z1);
     float C = (x2-x1)*(y3-y1) -(y2-y1)*(x3-x1);
     float D = -(A*x1 + B*y1 + C*z1);

     for(int i = 0; i < cloud->points.size(); i++)
     {
       if (inliers.count(i)>0)
       {
         continue;
       }
       PointT point = cloud->points[i];
       float x_x = point.x;
       float y_x = point.y;
       float z_x = point.z;

       float distance3D = fabs(A*x_x + B*y_x + C*z_x + D)/sqrt(A*A + B*B + C*C);

       if (distance3D <= distanceTol)
       {
         inliers.insert(i);
       }
     }
     if(inliers.size() > inliersResult.size())
     {
       inliersResult = inliers;
     }
   }
  
  
    typename pcl::PointCloud<PointT>::Ptr cloudPlane(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudObstacle(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudPlane->points.push_back(point);
		else
			cloudObstacle->points.push_back(point);
	}
 

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudObstacle, cloudPlane);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}



template <typename PointT>
void ProcessPointClouds<PointT>::proximity(const std::vector<std::vector<float>> &points, std::vector<int> &cluster, std::vector<bool> &processed,const int index, const float tolerance, KdTree *tree) 
{
    processed.at(index) = true;
    cluster.push_back(index);

    const std::vector<int> nearbyPoints{ tree->search(points.at(index), tolerance) };

    for (int nearbyIndex : nearbyPoints) 
    {
        if (!processed.at(nearbyIndex))
        {
            proximity(points, cluster, processed, nearbyIndex, tolerance, tree);
        }
    }
}



template <typename PointT>
std::vector<std::vector<int>>ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol) 
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    for (int i{ 0 }; i < points.size(); i++) 
    {
        if (!processed.at(i)) 
        {
            std::vector<int> cluster;
            proximity(points, cluster, processed, i, distanceTol, tree);
            clusters.push_back(cluster);
        }
    }
    return clusters;
}



template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, const float clusterTolerance, const int minSize, const int maxSize) 
{
   const auto startTime{ std::chrono::steady_clock::now() };
   int i = 0;

    KdTree *kdTree = new KdTree;

    std::vector<std::vector<float>> points;

    for (auto point : cloud->points)
    {
        const std::vector<float> p{ point.x, point.y, point.z };
        kdTree->insert(p, i++);
        points.push_back(p);
    }


    const std::vector<std::vector<int>> listOfIndices{ euclideanCluster(points, kdTree, clusterTolerance) };
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for (auto indices : listOfIndices) 
    {

        if (indices.size() < minSize || indices.size() > maxSize) 
        {
          continue; 
        }
      
        typename pcl::PointCloud<PointT>::Ptr cluster{ new pcl::PointCloud<PointT> };
        for (auto index : indices) 
        { 
          cluster->points.push_back(cloud->points[index]); 
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }

    const auto endTime{ std::chrono::steady_clock::now() };
    const auto elapsedTime{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime) };
    std::cout << "clustering took " << elapsedTime.count();
    std::cout << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
// END OF MY ALGORITHM



template <typename PointT>
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



template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) 
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}



template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) 
{
	typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
        PCL_ERROR("Couldn't read file \n");

    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}



template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) 
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{ dataPath }, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}
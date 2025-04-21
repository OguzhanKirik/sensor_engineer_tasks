// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
  
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new  pcl::PointCloud<PointT>);

  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (filterRes, filterRes, filterRes);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
  
  typename pcl::PointCloud<PointT>::Ptr cloud_region(new  pcl::PointCloud<PointT>);
  
  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloud_filtered);
  region.filter(*cloud_region); 
  
    std::cerr << "PointCloud after crop boxing: " << cloud_region->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_region) << ")." << std::endl;
  
  std::vector<int> indices;
  //extract roof
  pcl::CropBox<PointT> roof;
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloud_region);
  roof.filter(indices);
  
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for(int indice : indices){
  	inliers->indices.push_back(indice);
  }
  // remove roof points
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_region);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_region);
  
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{

  
  typename pcl::PointCloud<PointT>::Ptr cloud_obstacle(new pcl::PointCloud<PointT>()); 
  typename pcl::PointCloud<PointT>::Ptr cloud_road(new pcl::PointCloud<PointT>()); 

  for (int index : inliers->indices){
    cloud_road->points.push_back(cloud->points[index]);
  }
  
  pcl::ExtractIndices<PointT> extract;

      // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_obstacle);

  
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_road, cloud_obstacle);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
  
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
  
  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
 
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0){
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
  }
  
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Create a KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);
  
  
    for (auto it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        // Create a cluster with the extracted points belonging to the same object
        for (auto pointIt = it->indices.begin(); pointIt != it->indices.end(); ++pointIt)
        {
            cluster->push_back(cloud->points[*pointIt]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        // Add the cluster to the return cluster vector
        clusters.push_back(cluster);
    }
  
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
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






    


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::segmentRansacPlane(typename pcl::PointCloud<PointT>::Ptr inputCloud, int maxIterations,float distanceTol)
{
    auto startTime= std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

  
    while(maxIterations--)
    {
        std::unordered_set<int> inliers;
        while (inliers.size()<3)
            inliers.insert(rand()%(inputCloud->points.size()));
        float x1,y1,x2,y2,z1,z2,x3,y3, z3;

        auto itr = inliers.begin();
        x1 = inputCloud->points[*itr].x;
        y1 = inputCloud->points[*itr].y;
        z1 = inputCloud->points[*itr].z;
        itr ++;
        x2 = inputCloud->points[*itr].x;
        y2 = inputCloud->points[*itr].y;
        z2 = inputCloud->points[*itr].z;
        itr ++;
        x3 = inputCloud->points[*itr].x;
        y3 = inputCloud->points[*itr].y;
        z3 = inputCloud->points[*itr].z;

        std::vector<float> v1 = {(x2-x1),(y2-y1),(z2-z1)};
        std::vector<float> v2 = {(x3-x1),(y3-y1),(z3-z1)};

        std::vector<float> cros_prod = {(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1),
                                        (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1),
                                        (x2-x1)*(y3-y1)-(y2-y1)*(x2-x1)};

        float ii = cros_prod[0];
        float jj = cros_prod[1];
        float kk = cros_prod[2];
        std::complex<float> i,j,k;
        std::complex<float> d = -(x1*ii + jj*y1 + kk*z1);

        for (int index = 0 ; index <inputCloud->points.size() ; index ++)
        {
            if (inliers.count(index)>0)
                continue;
             PointT point = inputCloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            float dist = fabs(ii*x4+jj*y4+kk*z4+d)/sqrt(ii*ii +jj*jj+kk*kk);
            if (dist <= distanceTol)
                inliers.insert(index);
        }
        if (inliers.size()>inliersResult.size())
            inliersResult= inliers;

    }


    auto endTime= std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (auto i : inliersResult) {
        inliers->indices.push_back(i);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,inputCloud);

    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree3D* tree, float distanceTol )
{
    processed[indice] =true;
    cluster.push_back(indice);

    std::vector<int> nearest = tree->search(points[indice],distanceTol);

    for (int id :nearest)
    {
        if (!processed[id])
            clusterHelper(id,points,cluster,processed,tree,distanceTol);
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree3D* tree, float distanceTol)
{

    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(),false);

    int i = 0;
    while (i<points.size())
    {
        if (processed[i])
        {
            i++;
            continue;
        }
        std::vector<int>cluster;
        clusterHelper(i,points,cluster,processed,tree,distanceTol);
        clusters.push_back(cluster);
        i++;
    }

    return clusters;
}



template<typename PointT>
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>ProcessPointClouds<PointT>::euclideanClustering(typename pcl::PointCloud<PointT>::Ptr inputCloud,  float distanceTol, int minSize, int maxSize)
{
  
    auto startTime= std::chrono::steady_clock::now();

    KdTree3D* tree = new KdTree3D;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    std::vector<std::vector<float>> points;
    for (int i=0; i< inputCloud->points.size(); i++)
    {
        std::vector<float> point({inputCloud->points[i].x, inputCloud->points[i].y, inputCloud->points[i].z});
        points.push_back(point);
        tree->insert(points[i],i);
    }

    std::vector<std::vector<int>> indicies = euclideanCluster(points, tree, distanceTol);

    for (const auto&  indice: indicies)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster ( new pcl::PointCloud<PointT>);
        for (const auto index : indice)
            cluster->points.push_back(inputCloud->points[index]);

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        if(cluster->width >= minSize && cluster->width <= maxSize)
        {
            clusters.push_back(cluster);
        }
        auto endTime= std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
       std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    }

    return clusters;
}
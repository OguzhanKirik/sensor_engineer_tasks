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

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  	auto startTime= std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
  
  	while(maxIterations--)
    {
      
     	//Randomly pick two points
      
    	std::unordered_set<int> inliers;
  		while (inliers.size() <2)
   			inliers.insert(rand()%(cloud ->points.size()));
  
  		float x1 , y1 ,z1 , x2, y2 , z2 , x3 , y3 , z3;
  
  		auto itr = inliers.begin();
  		x1 = cloud ->points[*itr].x;
  		y1 = cloud ->points[*itr].y;
      	z1 = cloud ->points[*itr].z;
  		itr++;
  		x2 = cloud ->points[*itr].x;
  		y2 = cloud ->points[*itr].y;
  		z2 = cloud ->points[*itr].z;     
  		itr++;
  		x3 = cloud ->points[*itr].x;
  		y3 = cloud ->points[*itr].y; 
  		z3 = cloud ->points[*itr].z;      


      	// the the vector v1 is: {(x2-x1),(y2-y1),(z2-z1)}
      	// the the vector v2 is: {(x3-x1),(y3-y1),(z3-z1)}     
      
		// the cross product from v1 and v2 is: {i,j,k} ; where:

        float i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
        float j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
        float k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1); 
      
        std::vector<float> crossProduct_v1_v2 = {i,j,k};
      
      	// the Plane equation is Ax+By+Cz+D=0 ; where:
      
      	float eq_A = i;
      	float eq_B = j;
      	float eq_C = k;
      	float eq_D = -(eq_A*x1 + eq_B*y1 + eq_C*z1);      
      

  
  		for (int index = 0; index < cloud -> points.size(); index++)
  		{
   			if (inliers.count(index)>0)
     			 continue;
    		pcl::PointXYZ point = cloud ->points [index];
    		float x4 = point.x;
    		float y4 = point.y;
    		float z4 = point.z;
          
            //The distance from the point to the Plane is |∣A∗x+B∗y+C∗z+D∣/sqrt(A*A + B*B + C*C)

    		float d = (fabs(eq_A*x4+eq_B*y4+eq_C*z4+eq_D))/(sqrt(eq_A*eq_A+ eq_B*eq_B+eq_C*eq_C));
    
    		if (d<= distanceTol)
      			inliers.insert(index);
  		}
    	if (inliers.size()>inliersResult.size())
  		{
  			inliersResult = inliers; 
  		}
    }
	auto endTime = std::chrono::steady_clock::now();
    	// Time segmentation process
  
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac took" << elapsedTime.count() << "milliseconds" << std::endl;
  

	return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  	auto startTime= std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
  
  	while(maxIterations--)
    {
      
     	//Randomly pick two points
      
    	std::unordered_set<int> inliers;
  		while (inliers.size() <2)
   			inliers.insert(rand()%(cloud ->points.size()));
  
  		float x1 , y1 , x2, y2;
  
  		auto itr = inliers.begin();
  		x1 = cloud ->points[*itr].x;
  		y1 = cloud ->points[*itr].y;
  		itr++;
  		x2 = cloud ->points[*itr].x;
  		y2 = cloud ->points[*itr].y;
  
  		float a = (y1 - y2);
  		float b = (x2 - x1);
  		float c = (x1*y2 - x2*y1);
  
  		for (int index = 0; index < cloud -> points.size(); index++)
  		{
   			if (inliers.count(index)>0)
     			 continue;
    		pcl::PointXYZ point = cloud ->points [index];
    		float x3 = point.x;
    		float y3 = point.y;
    
    		float d = fabs (a*x3+b*y3+c)/sqrt(a*a+b*b);
    
    		if (d<= distanceTol)
      			inliers.insert(index);
  		}
    	if (inliers.size()>inliersResult.size())
  		{
  			inliersResult = inliers; 
  		}
    }
	auto endTime = std::chrono::steady_clock::now();
    	// Time segmentation process
  
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac took" << elapsedTime.count() << "milliseconds" << std::endl;
  

	return inliersResult;
}

std::unordered_set<int> RansacPlane_(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime= std::chrono::steady_clock::now();

    std::unordered_set<int> bestInliersResult;
    srand(time(NULL));

    while (maxIterations--)
    {
        std::unordered_set<int> inliers;

        // Randomly pick 2 points
        while (inliers.size() < 2)
            inliers.insert(rand() % cloud->points.size());

        auto itr = inliers.begin();
        pcl::PointXYZ p1 = cloud->points[*itr];
        itr++;
        pcl::PointXYZ p2 = cloud->points[*itr];

        // Line coefficients A*x + B*y + C = 0
        float A = p1.y - p2.y;
        float B = p2.x - p1.x;
        float C = (p1.x * p2.y) - (p2.x * p1.y);

        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (inliers.count(i) > 0)
                continue;

            pcl::PointXYZ p = cloud->points[i];

            float distance = fabs(A * p.x + B * p.y + C) / sqrt(A * A + B * B);

            if (distance <= distanceTol)
                inliers.insert(i);
        }

        if (inliers.size() > bestInliersResult.size())
            bestInliersResult = inliers;
    }

    return bestInliersResult;
  
  	auto endTime = std::chrono::steady_clock::now();  
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac took" << elapsedTime.count() << "milliseconds" << std::endl;
  
}



int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 0, 0);
    // std::unordered_set<int> inliers = RansacLine1(cloud, 50, 0.5);
    //std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.2);
    std::unordered_set<int> inliers = RansacPlane_(cloud, 50, 0.2);
	
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

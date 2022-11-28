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
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	
	// For max iterations

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	for (int i = 0; i < maxIterations; i++)
	{
		std::unordered_set<int> inliers;
		while(inliers.size() < 2)
			inliers.insert(rand() % (cloud->points.size()));

		float x1, y1, x2, y2;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float a = y1 - y2;
		float b = x2 - x1;
		float c = x1 * y2 - x2 * y1;

		for(int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0)
			{
				continue;
			}
			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;
			double dist = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b);
			

			if(dist <= distanceTol)
				inliers.insert(index);

		}

		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	
	// For max iterations

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	for (int i = 0; i < maxIterations; i++)
	{
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand() % (cloud->points.size()));

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
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

		float v1[] = {x2 - x1, y2 - y1, z2 - z1};
		float v2[] = {x3 - x1, y3 - y1, z3 - z1};
		float v_cross[3]; // = {v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0]};
		for(int i = 0; i < 3; i++)
		{
			v_cross[i] = v1[(i+1) % 3] * v2[(i+2) % 3] - v2[(i+1) % 3] * v1[(i+2) % 3];
		}

		float a = v_cross[0];
		float b = v_cross[1];
		float c = v_cross[2];
		float d = -1 * (a * x1 + b * y1 + c * z1);

		for(int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0)
			{
				continue;
			}

			pcl::PointXYZ point = cloud->points[index];

			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			
			double dist = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

			if(dist <= distanceTol)
				inliers.insert(index);
		}

		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::unordered_set<int> inliers;
	ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>;
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());
	
	enum Dim
	{
		D2, D3
	};

	int maxItr = 500;
	float distTol = 0.2;

	Dim cloud_type = D3;
	
	switch (cloud_type)
	{
		case D2:
		{
			// Create data
			cloud = CreateData();
			// TODO: Change the max iteration and distance tolerance arguments for Ransac function
			inliers = Ransac(cloud, maxItr, distTol); //0, 0

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
				renderPointCloud(viewer,cloud,"data");
			break;
		}
			
		case D3:
		{
			// Create data
			cloud = CreateData3D();
			// TODO: Change the max iteration and distance tolerance arguments for Ransac function
			inliers = RansacPlane(cloud, maxItr, distTol); //0, 0
			
			for(int index = 0; index < cloud->points.size(); index++)
			{
				pcl::PointXYZ point = cloud->points[index];
				if(inliers.count(index))
					cloudInliers->points.push_back(point);
				else
					cloudOutliers->points.push_back(point);
			}

			// Render 3D point cloud with inliers and outliers
			if(inliers.size())
			{
				renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
				renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
			}
			else
				renderPointCloud(viewer,cloud,"data");


			/*
			std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor -> SegmentPlane(cloud, 500, 0.5);
			
			if(inliers.size())
			{
				renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    			renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
			}
			else
			{
				renderPointCloud(viewer,cloud,"data");
			}
			*/
			break;
		}	
	}

	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}

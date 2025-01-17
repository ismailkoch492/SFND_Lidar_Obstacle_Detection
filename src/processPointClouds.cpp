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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    
    pcl::VoxelGrid<PointT> fltr;
    fltr.setInputCloud(cloud);
    fltr.setLeafSize(filterRes, filterRes, filterRes);
    fltr.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
    
    pcl::CropBox<PointT> cboxRegion(true);
    cboxRegion.setMin(minPoint);
    cboxRegion.setMax(maxPoint);
    cboxRegion.setInputCloud(cloud_filtered);
    cboxRegion.filter(*cloud_region);

    std::vector<int> indices;

    pcl::CropBox<PointT> cboxRoof(true);
    cboxRoof.setMin(Eigen::Vector4f(-1.5, -1.7, -1  , 1));
    cboxRoof.setMax(Eigen::Vector4f( 2.6,  1.7, -0.4, 1));
    cboxRoof.setInputCloud(cloud_region);
    cboxRoof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int i: indices)
        inliers->indices.push_back(i);

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
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);
    // Create filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud); //cloud_filtered
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients()};
    // Create segmentation object
    pcl::SACSegmentation<PointT> seg;
    // optional
    seg.setOptimizeCoefficients(true);
    // necessary
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomRANSAC(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>);

    // template pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    // template pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());
    
	srand(time(NULL));
	
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

			PointT point = cloud->points[index];

			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			
			double dist = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

			if(dist <= distanceThreshold)
				inliers.insert(index);
		}

		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;


    for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
	
	return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree3D* tree, float distanceTol)
{
	processed[indice]= true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice], distanceTol);

	for(int id : nearest)
	{
		if(!processed[id])
			this->clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree3D* tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster
	// list of clusters
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);
	// Iterate through each point
	for(int i = 0; i < points.size(); i++)
	{	
		// If point has not been processed
		if(!processed[i])
		{
			// Create cluster
			std::vector<int> cluster;
			// Proximity(point, cluster)
			clusterHelper(i, points, cluster, processed, tree, distanceTol);
			// cluster add clusters
			clusters.push_back(cluster);
		}
	}
 
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    std::vector<std::vector<float>> points;

    for (const auto& point : *cloud)
        points.push_back({point.x, point.y, point.z});

    KdTree3D* tree = new KdTree3D;

    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i);

    std::vector<std::vector<int>> clusters_ids = euclideanCluster(points, tree, clusterTolerance);
    

    for (const auto& i : clusters_ids)
	{
        typename pcl::PointCloud<PointT>::Ptr cloud_(new pcl::PointCloud<PointT>());
        
        if (i.size() < minSize || i.size() > maxSize)
            continue;
        
        for (auto j : i)
        {
            PointT point_;
            point_.x = points[j][0];
            point_.y = points[j][1];
            point_.z = points[j][2];

            cloud_->points.push_back(point_);
        }
        cloud_->width = cloud_->points.size();
        cloud_->height = 1;
        clusters.push_back(cloud_);
	}
    // point clouds in clusters are cleared when the for loop ended

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
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
BoxQ ProcessPointClouds<PointT>::PCABoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Source: http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
    
    // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
    typename pcl::PointCloud<PointT>::Ptr cloudPCAprojection (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloudPCAreconst (new pcl::PointCloud<PointT>);
    pcl::PCA<PointT> pca, reconst;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *cloudPCAprojection);
    pca.reconstruct(*cloudPCAprojection, *cloudPCAreconst);
    //std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    //std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
    // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.

    Eigen::Vector4f centPCA;
    pcl::compute3DCentroid(*cluster, centPCA);
    Eigen::Matrix4f projTran(Eigen::Matrix4f::Identity());
    projTran.block<3,3>(0,0) = pca.getEigenVectors().transpose();
    projTran.block<3,1>(0,3) = -1.f * (projTran.block<3,3>(0,0) * centPCA.head<3>());
    typename pcl::PointCloud<PointT>::Ptr projPCL (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *projPCL, projTran);

    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*projPCL, minPoint, maxPoint);
    const Eigen::Vector3f meanDiag = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    const Eigen::Quaternionf pcaQuaternion(pca.getEigenVectors());
    const Eigen::Vector3f pcaTransform = pca.getEigenVectors() * meanDiag + centPCA.head<3>();

    BoxQ box;
    box.cube_length = std::abs(minPoint.x - maxPoint.x);
    box.cube_height = std::abs(minPoint.z - maxPoint.z);
    box.cube_width = std::abs(minPoint.y - maxPoint.y);
    box.bboxTransform  = pcaTransform;
    box.bboxQuaternion = pcaQuaternion;

    return box;
    
    /*
    Eigen::Vector4f centPCA;
    pcl::compute3DCentroid(*cluster, centPCA);
    Eigen::Matrix3f cov;
    pcl::computeCovarianceMatrixNormalized(*cluster, centPCA, cov);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(cov, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigVecsPCA = eigen_solver.eigenvectors();
    eigVecsPCA.col(2) = eigVecsPCA.col(0).cross(eigVecsPCA.col(1));

    Eigen::Matrix4f projTran(Eigen::Matrix4f::Identity());
    projTran.block<3,3>(0,0) = eigVecsPCA.transpose();
    projTran.block<3,1>(0,3) = -1.f * (projTran.block<3,3>(0,0) * centPCA.head<3>());
    typename pcl::PointCloud<PointT>::Ptr projPCL (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *projPCL, projTran);

    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*projPCL, minPoint, maxPoint);
    const Eigen::Vector3f meanDiag = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    const Eigen::Quaternionf pcaQuaternion(eigVecsPCA);
    const Eigen::Vector3f pcaTransform = eigVecsPCA * meanDiag + centPCA.head<3>();
    */
    /*box.h
    struct BoxQ
    {
        Eigen::Vector3f bboxTransform;
        Eigen::Quaternionf bboxQuaternion;
        float cube_length;
        float cube_width;
        float cube_height;
    };
    */
    /*
    BoxQ box;
    box.cube_length = std::abs(minPoint.x - maxPoint.x);
    box.cube_height = std::abs(minPoint.z - maxPoint.z);
    box.cube_width = std::abs(minPoint.y - maxPoint.y);
    box.bboxTransform  = pcaTransform;
    box.bboxQuaternion = pcaQuaternion;
    
    return box;
    */
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
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath, char** argv)
{
    boost::filesystem::path complete_path(boost::filesystem::system_complete(argv[0]));
    boost::filesystem::path data_path;
    std::vector<std::string> folders; 
    std::string token;
    size_t pos = 0;
    
    // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
    while ((pos = dataPath.find("/")) != std::string::npos)
    {
        token = dataPath.substr(0, pos);
        folders.push_back(token);
        dataPath.erase(0, pos + 1);
    }
    folders.push_back(dataPath);
    
    // https://www.boost.org/doc/libs/1_77_0/libs/filesystem/doc/tutorial.html
    // https://www.boost.org/doc/libs/1_68_0/libs/filesystem/doc/reference.html
    // https://www.boost.org/doc/libs/1_45_0/libs/filesystem/v3/doc/reference.html#path-appends
    // https://theboostcpplibraries.com/boost.filesystem-paths#ex.filesystem_08
    for(const boost::filesystem::path &pp : complete_path)
    {
        if(pp.string() == "build")
            break;
        if(pp.string() == "/" || pp.string() == ".")
            continue;
        data_path /= "/";
        data_path /= pp;
    }

    for(const auto &f : folders)
    {
        if(f == "..")
            continue;
        data_path /= "/";
        data_path /= f;
    }
    
    std::cout << "Current directory: " << boost::filesystem::current_path() << "\n";
    std::cout << "Data directory: " << data_path << "\n";
    
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{data_path}, boost::filesystem::directory_iterator{});
    
    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
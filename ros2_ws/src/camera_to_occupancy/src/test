#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

int main() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>(std::string(CMAKE_PROJECT_DATA_DIRECTORY) + std::string("/test_cloud.pcd"), *cloud);

	// VoxelGrid filtering
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	cloud = cloud_filtered;

	// Pass trough filter
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.5, 4.0);
	pass.filter(*cloud_filtered);
	*cloud = *cloud_filtered;

	// Rigid transform cloud
	// Find min point
	
	// Change orientation
	Eigen::Matrix3d camera2image = Eigen::Matrix3d::Zero();
	camera2image(0, 2) =  1.0;
	camera2image(1, 0) = -1.0;
 	camera2image(2, 1) = -1.0;

	Eigen::Affine3d odom2drone = Eigen::Affine3d::Identity();
 	odom2drone.linear() = camera2image;
 	pcl::transformPointCloud(*cloud, *cloud, odom2drone);
	
	double min = 100.0; // Big Number
	for (auto& point : *cloud) {
		if (point.z < min)
			min = point.z;
	}
	std::cout << "Min is : " << min << std::endl;

	odom2drone = Eigen::Affine3d::Identity();
	odom2drone.translation() = Eigen::Vector3d(0.0, 0.0, -min);
 	pcl::transformPointCloud(*cloud, *cloud, odom2drone);
	
	// Remove ceiling
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr ceiling_condition(new pcl::ConditionAnd<pcl::PointXYZ> ());
	ceiling_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 2.5)));
	pcl::ConditionalRemoval<pcl::PointXYZ> ceiling_remover;
	ceiling_remover.setCondition(ceiling_condition);
	ceiling_remover.setInputCloud(cloud);
	ceiling_remover.setKeepOrganized(true);
	ceiling_remover.filter(*cloud_filtered);
	*cloud = *cloud_filtered;
	
	// Remove Ground 2
	pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.05);

	seg.setInputCloud(cloud);
	seg.segment(*plane_inliers, *plane_coefficients);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(plane_inliers);
	extract.setNegative(true);
	extract.filter(*cloud_filtered);
	*cloud = *cloud_filtered;
	
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(20);
	sor.setStddevMulThresh(0.5);
	sor.filter(*cloud_filtered);
	*cloud = *cloud_filtered;

	// Extract Objects
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_clusters;
	euclidean_clusters.setClusterTolerance(0.02);
	euclidean_clusters.setMinClusterSize(250);
	euclidean_clusters.setMaxClusterSize(25000);
	euclidean_clusters.setSearchMethod(tree);
	euclidean_clusters.setInputCloud(cloud);
	euclidean_clusters.extract(cluster_indices);

	// Prepare projection
	pcl::ModelCoefficients::Ptr projection_coefficients(new pcl::ModelCoefficients());
	projection_coefficients->values.resize(4);
	projection_coefficients->values[0] = projection_coefficients->values[1] = 0.0;
	projection_coefficients->values[2] = 1.0;
	projection_coefficients->values[3] = 0.0;
	pcl::ProjectInliers<pcl::PointXYZ> projection;
	projection.setModelType(pcl::SACMODEL_PLANE);
	projection.setModelCoefficients(projection_coefficients);

	pcl::PCDWriter writer;
	int j=0;
	for (const auto& cluster : cluster_indices) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (const auto& idx : cluster.indices) {
			cloud_cluster->push_back((*cloud)[idx]);
		}

		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		writer.write<pcl::PointXYZ>(std::string(CMAKE_PROJECT_DATA_DIRECTORY) + std::string("/ldc/test_cloud_") + std::to_string(j) + std::string(".pcd"), *cloud_cluster, false);		
		projection.setInputCloud(cloud_cluster);
		projection.filter(*cloud_cluster);

		writer.write<pcl::PointXYZ>(std::string(CMAKE_PROJECT_DATA_DIRECTORY) + std::string("/ldc/test_cloud_projected_") + std::to_string(j) + std::string(".pcd"), *cloud_cluster, false);		
		j++;
	}

	return(0);
}

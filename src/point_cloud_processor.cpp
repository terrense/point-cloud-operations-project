#include "point_cloud_processor.h"
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>

PointCloudProcessor::PointCloudProcessor() {
    // Constructor implementation if needed
}

void PointCloudProcessor::segmentPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Segment the point cloud into different parts
    // This is a placeholder for segmentation algorithm, can be extended as needed
}

void PointCloudProcessor::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Apply a PassThrough filter to remove NaNs and limit the region of interest
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.5);
    pass.filter(*cloud);

    // Apply a StatisticalOutlierRemoval filter to remove noise
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);
}

void PointCloudProcessor::clusterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Euclidean Cluster Extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm tolerance
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Process clusters (e.g., color each cluster differently)
    int j = 0;
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : indices.indices)
            cluster->points.push_back(cloud->points[idx]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        std::cout << "Cluster " << j << " has " << cluster->points.size() << " points." << std::endl;
        j++;
    }
}

void PointCloudProcessor::separateGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    removeGroundPlane(cloud);
}

void PointCloudProcessor::removeGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Plane segmentation to separate ground
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);

    seg.setInputCloud(cloud);
    seg.segment(*ground_indices, *coefficients);

    if (ground_indices->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground_indices);
    extract.setNegative(false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*ground_cloud);

    // Optionally save the ground point cloud to a file
    pcl::io::savePCDFileASCII("ground.pcd", *ground_cloud);

    // Remove ground points from the original cloud
    extract.setNegative(true);
    extract.filter(*cloud);
}

#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

class PointCloudProcessor {
public:
    PointCloudProcessor();

    /**
     * @brief Segments the point cloud into meaningful segments.
     * @param cloud The input point cloud to be segmented.
     */
    void segmentPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief Filters the point cloud to remove noise and outliers.
     * @param cloud The input point cloud to be filtered.
     */
    void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief Clusters the point cloud into different groups.
     * @param cloud The input point cloud to be clustered.
     */
    void clusterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief Separates the ground points from the point cloud.
     * @param cloud The input point cloud from which the ground points will be separated.
     */
    void separateGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
    /**
     * @brief Removes points from the ground plane using RANSAC.
     * @param cloud The input point cloud from which the ground points will be removed.
     */
    void removeGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif // POINT_CLOUD_PROCESSOR_H

#ifndef POINT_CLOUD_VISUALIZER_H
#define POINT_CLOUD_VISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudVisualizer {
public:
    PointCloudVisualizer();

    /**
     * @brief Visualizes the given point cloud data.
     * @param cloud The input point cloud to be visualized.
     */
    void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif // POINT_CLOUD_VISUALIZER_H

#ifndef MAPPING_H
#define MAPPING_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Mapping {
public:
    Mapping();

    /**
     * @brief Creates a simple map from the processed point cloud data.
     * @param cloud The input point cloud from which the map will be created.
     */
    void createMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif // MAPPING_H

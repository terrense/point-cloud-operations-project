#include "mapping.h"
#include <iostream>

Mapping::Mapping() {
    // Constructor implementation if needed
}

void Mapping::createMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Simple mapping implementation
    // This is a placeholder for the mapping algorithm, can be extended as needed
    std::cout << "Creating a simple map from the point cloud data." << std::endl;

    // Example: Calculate and print the bounding box of the point cloud
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    std::cout << "Bounding box:" << std::endl;
    std::cout << "Min: (" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ")" << std::endl;
    std::cout << "Max: (" << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << ")" << std::endl;
}

#include "mapping.h"
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

Mapping::Mapping() {
    // Constructor implementation if needed
}

void Mapping::createMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::cout << "Creating a simple map from the point cloud data." << std::endl;

    // Example: Calculate and print the bounding box of the point cloud
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    std::cout << "Bounding box:" << std::endl;
    std::cout << "Min: (" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ")" << std::endl;
    std::cout << "Max: (" << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << ")" << std::endl;

    // Additional mapping logic can be added here
    // For instance, creating a grid map, occupancy map, or 3D mesh from the point cloud

    // Example: Save the processed point cloud as a map
    pcl::io::savePCDFileASCII("map.pcd", *cloud);
    std::cout << "Map saved as map.pcd" << std::endl;
}

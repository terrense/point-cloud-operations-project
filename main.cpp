
### src/main.cpp

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include "point_cloud_processor.h"
#include "point_cloud_visualizer.h"
#include "mapping.h"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_point_cloud.pcd>" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", input_file.c_str());
        return -1;
    }

    PointCloudProcessor processor;
    processor.filterPointCloud(cloud);
    processor.separateGround(cloud);
    processor.segmentPointCloud(cloud);
    processor.clusterPointCloud(cloud);

    Mapping mapping;
    mapping.createMap(cloud);

    PointCloudVisualizer visualizer;
    visualizer.visualize(cloud);

    // Optionally save the processed point cloud to a file
    pcl::io::savePCDFileASCII("processed_cloud.pcd", *cloud);

    return 0;
}

# point-cloud-operations-project
A simple project about how operate point cloud

# Advanced Indoor Dense Point Cloud Processing using Visual SLAM

This project demonstrates advanced processing of dense point cloud data obtained from visual SLAM in indoor environments. The main steps include segmentation, filtering, and clustering of the point cloud data, with additional functionalities like ground removal, mapping, and visualization.

## Features
- **Segmentation:** Divide the point cloud into meaningful segments.
- **Filtering:** Apply filters to remove noise and outliers.
- **Clustering:** Group similar points together.
- **Ground Separation:** Specifically identify and separate ground points.
- **Mapping:** Create a simple map from the processed point cloud data.
- **Visualization:** Visualize the processed point cloud data.

## Requirements
- C++17 or later
- PCL (Point Cloud Library)
- OpenCV
- Eigen
- VTK (for visualization)

## Setup

1. Install the required libraries:
   ```bash
   sudo apt-get update
   sudo apt-get install libpcl-dev libopencv-dev libeigen3-dev libvtk7-dev

#include "Task.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>

namespace pcl_io_preprocessing {

bool Task::configureHook(void) {
    if (!TaskBase::configureHook()) return false;

    return true;
}

void Task::updateHook(void) {
    TaskBase::updateHook();

    static bool loaded = false;
    if (loaded) return;
    loaded = true;

    // BaseCloud baseCloud;
    Cloud::Ptr cloud(new Cloud);

    std::cout << "Reading ply..." << std::endl;
    // pcl::io::loadPLYFile<pcl::PointXYZ>(
        // "~/rock_bags/ebee/simplified_mesh.ply", *cloud);
    pcl::io::loadPLYFile<pcl::PointXYZ>(
        "~/rock_bags/ebee/densified_cloud.ply", *cloud);
    std::cout << "Cloud size: " << cloud->width << std::endl;

    std::cout << "Downsampling cloud..." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(1., 1., 1.);
    voxelGrid.filter(*cloud);
    std::cout << "New size: " << cloud->width << std::endl;

    // std::cout << "Converting to base cloud..." << std::endl;
    // GaSlamBaseConverter::convertPCLToBaseCloud(baseCloud, cloud);

    // std::cout << "Writing to port..." << std::endl;
    // _mapCloud.write(baseCloud);

    std::cout << "Done!" << std::endl;
}

}  // namespace pcl_io_preprocessing


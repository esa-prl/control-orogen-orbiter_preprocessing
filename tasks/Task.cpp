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

    BaseCloud baseCloud;
    Cloud::Ptr cloud(new Cloud);

    std::cout << "Reading ply..." << std::endl;
    pcl::io::loadPLYFile<pcl::PointXYZ>(_plyFilename.rvalue(), *cloud);
    std::cout << "Cloud size: " << cloud->width << std::endl;

    std::cout << "Downsampling cloud..." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(1., 1., 1.);
    voxelGrid.filter(*cloud);
    std::cout << "New size: " << cloud->width << std::endl;

    std::cout << "Converting to base cloud..." << std::endl;
    baseCloud.points.clear();
    baseCloud.points.reserve(cloud->size());
    baseCloud.time.fromMicroseconds(cloud->header.stamp);

    for (const auto& point : cloud->points)
        baseCloud.points.push_back(base::Point(point.x, point.y, point.z));

    std::cout << "Finished configuring!" << std::endl;

    std::cout << "Writing to port..." << std::endl;
    _pointCloud.write(baseCloud);
    std::cout << "Done!" << std::endl;
}

}  // namespace pcl_io_preprocessing


#include "Task.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

namespace pcl_io_preprocessing {

Task::Task(std::string const& name)
        : TaskBase(name),
          initialized_(false) {
    cloud_.reset(new Cloud);
}

void Task::updateHook(void) {
    TaskBase::updateHook();

    if (initialized_) return;

    loadCloud();
    preprocessCloud();
    writeCloud();

    initialized_ = true;
}

void Task::loadCloud(void) {
    std::cout << "Reading ply..." << std::endl;

    pcl::io::loadPLYFile<pcl::PointXYZ>(_plyFilename.rvalue(), *cloud_);
    std::cout << "Size: " << cloud_->width << std::endl;
}

void Task::preprocessCloud(void) {
    std::cout << "Downsampling cloud..." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud_);
    voxelGrid.setLeafSize(1., 1., 1.);
    voxelGrid.filter(*cloud_);
    std::cout << "Size: " << cloud_->width << std::endl;

    std::cout << "Transforming cloud..." << std::endl;
    const auto translation = Eigen::Vector3d(0., 0., 0.);
    const auto rotation = Eigen::Vector3d(0., 0., 0.);
    const auto quaternion = Eigen::Quaterniond(
            Eigen::AngleAxisd(rotation(2), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotation(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotation(0), Eigen::Vector3d::UnitX()));
    auto tf = Eigen::Affine3d::Identity();
    tf.translation() = translation;
    tf.linear() = quaternion.toRotationMatrix();
    pcl::transformPointCloud(*cloud_, *cloud_, tf);

    std::cout << "Cropping cloud..." << std::endl;
    Eigen::Vector4f minCutoffPoint(-100., -100., -1000., 0.);
    Eigen::Vector4f maxCutoffPoint(100., 100., 1000., 0.);
    pcl::CropBox<pcl::PointXYZ> cropBox;
    cropBox.setInputCloud(cloud_);
    cropBox.setMin(minCutoffPoint);
    cropBox.setMax(maxCutoffPoint);
    cropBox.filter(*cloud_);
    std::cout << "Size: " << cloud_->width << std::endl;
}

void Task::writeCloud(void) {
    std::cout << "Converting to base cloud..." << std::endl;

    BaseCloud baseCloud;
    baseCloud.points.clear();
    baseCloud.points.reserve(cloud_->size());
    baseCloud.time.fromMicroseconds(cloud_->header.stamp);

    for (const auto& point : cloud_->points)
        baseCloud.points.push_back(base::Point(point.x, point.y, point.z));

    std::cout << "Writing to port..." << std::endl;
    _pointCloud.write(baseCloud);
    std::cout << "Done!" << std::endl;
}

}  // namespace pcl_io_preprocessing


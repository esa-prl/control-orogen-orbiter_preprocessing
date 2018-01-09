#include "Task.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
    const auto filename = _plyFilename.value();
    pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud_);
}

void Task::preprocessCloud(void) {
    downsampleCloud();
    transformCloud();
    cropCloud();
    smoothCloud();
}

void Task::downsampleCloud(void) {
    const auto voxelSize = _voxelSize.value();

    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud_);
    voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxelGrid.filter(*cloud_);
}

void Task::transformCloud(void) {
    auto tf = Eigen::Affine3d::Identity();
    const auto rotation = _rotation.get();
    const auto quaternion = Eigen::Quaterniond(
            Eigen::AngleAxisd(rotation(2), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotation(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotation(0), Eigen::Vector3d::UnitX()));

    tf.translation() = _translation.get();
    tf.linear() = quaternion.toRotationMatrix();
    pcl::transformPointCloud(*cloud_, *cloud_, tf);
}

void Task::cropCloud(void) {
    const auto box = _box.get();
    const Eigen::Vector4f minCutoffPoint(-box(0)/2, -box(1)/2, -box(2)/2, 0.);
    const Eigen::Vector4f maxCutoffPoint(box(0)/2, box(1)/2, box(2)/2, 0.);

    pcl::CropBox<pcl::PointXYZ> cropBox;
    cropBox.setInputCloud(cloud_);
    cropBox.setMin(minCutoffPoint);
    cropBox.setMax(maxCutoffPoint);
    cropBox.filter(*cloud_);
}

void Task::smoothCloud(void) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_);
    sor.setMeanK(_sorMeanK.rvalue());
    sor.setStddevMulThresh(_sorStdMultiplier.rvalue());
    sor.filter(*cloud_);
}

void Task::writeCloud(void) {
    BaseCloud baseCloud;
    baseCloud.points.clear();
    baseCloud.points.reserve(cloud_->size());
    baseCloud.time = base::Time::now();

    for (const auto& point : cloud_->points)
        baseCloud.points.push_back(base::Point(point.x, point.y, point.z));

    _pointCloud.write(baseCloud);
}

}  // namespace pcl_io_preprocessing


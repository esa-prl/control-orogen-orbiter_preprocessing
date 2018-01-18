#include "Task.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <string>

namespace cloud_preprocessing {

Task::Task(std::string const& name)
        : TaskBase(name),
          initialized_(false) {
    cloud_.reset(new Cloud);
}

void Task::updateHook(void) {
    TaskBase::updateHook();

    if (initialized_) return;

    loadCloud();

    if (_preprocessingEnabled.rvalue()) preprocessCloud();

    writeCloud();

    if (_savePreprocessedCloud.rvalue()) saveCloud();

    initialized_ = true;
}

void Task::loadCloud(void) {
    const auto filename = _loadFilename.rvalue();
    const auto extension = filename.substr(filename.find_last_of(".") + 1);

    if (extension == "pcd")
        pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud_);
    else if (extension == "ply")
        pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud_);
    else
        throw std::runtime_error("Failed to load cloud. Unkown file format");
}

void Task::saveCloud(void) {
    const auto filename = _saveFilename.rvalue();
    const auto extension = filename.substr(filename.find_last_of(".") + 1);

    if (extension == "pcd")
        pcl::io::savePCDFile<pcl::PointXYZ>(filename, *cloud_, true);
    else if (extension == "ply")
        pcl::io::savePLYFile<pcl::PointXYZ>(filename, *cloud_, true);
    else
        std::cout << "Failed to save cloud. Unkown file format" << std::endl;
}

void Task::preprocessCloud(void) {
    cropCloud();
    downsampleCloud();
    transformCloud();
    smoothCloud();
}

void Task::cropCloud(void) {
    using Vector = Eigen::Vector3d;

    const Vector box = _box.get();
    const Vector rotation = _rotation.get();
    const auto quaternion = Eigen::Quaterniond(
            Eigen::AngleAxisd(rotation(2), Vector::UnitZ()) *
            Eigen::AngleAxisd(rotation(1), Vector::UnitY()) *
            Eigen::AngleAxisd(rotation(0), Vector::UnitX()));
    const Vector translation = _translation.get();
    const Vector center = quaternion.toRotationMatrix() * translation;

    const Eigen::Vector4f minCutoffPoint(
            center(0) - box(0) / 2., center(1) - box(1) / 2., -box(2) / 2., 0.);
    const Eigen::Vector4f maxCutoffPoint(
            center(0) + box(0) / 2., center(1) + box(1) / 2., box(2) / 2., 0.);

    pcl::CropBox<pcl::PointXYZ> cropBox;
    cropBox.setInputCloud(cloud_);
    cropBox.setMin(minCutoffPoint);
    cropBox.setMax(maxCutoffPoint);
    cropBox.filter(*cloud_);
}

void Task::downsampleCloud(void) {
    const double voxelSize = _voxelSize.value();

    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud_);
    voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxelGrid.filter(*cloud_);
}

void Task::transformCloud(void) {
    auto tf = Eigen::Affine3d::Identity();
    const Eigen::Vector3d rotation = _rotation.get();
    const auto quaternion = Eigen::Quaterniond(
            Eigen::AngleAxisd(rotation(2), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotation(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotation(0), Eigen::Vector3d::UnitX()));

    tf.translation() = _translation.get();
    tf.linear() = quaternion.toRotationMatrix();
    pcl::transformPointCloud(*cloud_, *cloud_, tf);
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

}  // namespace cloud_preprocessing


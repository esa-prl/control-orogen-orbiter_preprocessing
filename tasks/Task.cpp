#include "Task.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <string>
#include <cmath>

namespace cloud_preprocessing {

Task::Task(std::string const& name)
        : TaskBase(name),
          initialized_(false) {
    cloud_.reset(new Cloud);
}

bool Task::configureHook(void) {
    if (!TaskBase::configureHook()) return false;

    eastingReference_ = _robotEastingOffset.value() -
        _cloudEastingOffset.value();
    northingReference_ = _robotNorthingOffset.value() -
        _cloudNorthingOffset.value();
    elevationReference_ = _robotElevationOffset.value() -
        _cloudElevationOffset.value();

    return true;
}

bool Task::startHook(void) {
    if (!TaskBase::startHook()) return false;

    loadCloud();

    return true;
}

void Task::updateHook(void) {
    TaskBase::updateHook();

    if (_robotPose.read(robotPose_) != RTT::NewData || !isReadyToPreprocess())
        return;

    if (_preprocessingEnabled.rvalue()) preprocessCloud();

    writeCloud();
    if (_savePreprocessedCloud.rvalue()) saveCloud();
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

bool Task::isReadyToPreprocess(void) {
    if (initialized_) return false;

    initialized_ = true;
    return true;
}

void Task::preprocessCloud(void) {
    cropCloud();
    downsampleCloud();
    transformCloud();

    if (_smoothCloud.rvalue()) smoothCloud();
}

void Task::cropCloud(void) {
    using Vector = Eigen::Vector3d;

    const auto quaternion = Eigen::Quaterniond(
            Eigen::AngleAxisd(_cloudYawOffset.rvalue(), Vector::UnitZ()) *
            Eigen::AngleAxisd(0., Vector::UnitY()) *
            Eigen::AngleAxisd(0., Vector::UnitX()));
    const Vector translation(- northingReference_ - robotPose_.position.y(),
            eastingReference_ + robotPose_.position.x(), 0.);
    const Vector center = quaternion.toRotationMatrix() * translation;

    const Eigen::Vector4f minCutoffPoint(
            center(0) - _cropSize.rvalue() / 2.,
            center(1) - _cropSize.rvalue() / 2.,
            -std::numeric_limits<double>::max(), 0.);
    const Eigen::Vector4f maxCutoffPoint(
            center(0) + _cropSize.rvalue() / 2.,
            center(1) + _cropSize.rvalue() / 2.,
            std::numeric_limits<double>::max(), 0.);

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
    const double offsetYaw = _cloudYawOffset.value();
    auto offsetTF = Eigen::Affine3d::Identity();
    const auto offsetQuaternion = Eigen::Quaterniond(
            Eigen::AngleAxisd(offsetYaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()));

    offsetTF.translation() = Eigen::Vector3d(
            - northingReference_ - robotPose_.position.y(),
            eastingReference_ + robotPose_.position.x(),
            elevationReference_ + robotPose_.position.z());
    offsetTF.linear() = offsetQuaternion.toRotationMatrix();
    pcl::transformPointCloud(*cloud_, *cloud_, offsetTF);

    const double robotYaw = robotPose_.getYaw() - M_PI / 2.;
    auto robotTF = Eigen::Affine3d::Identity();
    const auto robotQuaternion = Eigen::Quaterniond(
            Eigen::AngleAxisd(robotYaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()));

    robotTF.translation() = Eigen::Vector3d::Zero();
    robotTF.linear() = robotQuaternion.toRotationMatrix();
    pcl::transformPointCloud(*cloud_, *cloud_, robotTF);
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


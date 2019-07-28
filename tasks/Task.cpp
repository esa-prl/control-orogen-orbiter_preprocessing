#include "Task.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <string>
#include <cmath>

namespace orbiter_preprocessing {

Task::Task(std::string const& name)
        : TaskBase(name) {
    cloud_.reset(new Cloud);
    preprocessedCloud_.reset(new Cloud);
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

    std::cout << "[Orbiter Preprocessing] Loading orbiter map..." << std::endl;
    loadCloud();

    return true;
}

void Task::updateHook(void) {
    TaskBase::updateHook();

    if (_preprocessingEnabled.rvalue()) {
        if (_robotPose.read(robotPose_) != RTT::NewData ||
                !isReadyToPreprocess())
            return;

        preprocessCloud();
        writeCloud(preprocessedCloud_);
        if (_savePreprocessedCloud.rvalue()) saveCloud();
    } else {
        writeCloud(cloud_);
    }
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
        pcl::io::savePCDFile<pcl::PointXYZ>(filename, *preprocessedCloud_,true);
    else if (extension == "ply")
        pcl::io::savePLYFile<pcl::PointXYZ>(filename, *preprocessedCloud_,true);
    else
        std::cout << "Failed to save cloud. Unkown file format" << std::endl;
}

bool Task::isReadyToPreprocess(void) {
    if (!lastPose_.hasValidPosition()) {
        lastPose_ = robotPose_;
        return true;
    }

    const double diffX = robotPose_.position.x() - lastPose_.position.x();
    const double diffY = robotPose_.position.y() - lastPose_.position.y();
    const double gridEdgeDistance = _cropSize.value() / 2.;
    const double diffThreshold = _diffDistanceRatio.value() * gridEdgeDistance;

    if (std::abs(diffX) > diffThreshold || std::abs(diffY) > diffThreshold) {
        lastPose_ = robotPose_;
        return true;
    } else {
        return false;
    }
}

void Task::preprocessCloud(void) {
    transformCloud();
    downsampleCloud();
    cropCloud();

    if (_smoothCloud.rvalue()) smoothCloud();
}

void Task::transformCloud(void) {
    const double offsetYaw = _cloudYawOffset.value();
    auto offsetTF = Eigen::Affine3d::Identity();
    const auto offsetQuaternion = Eigen::Quaterniond(
            Eigen::AngleAxisd(offsetYaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()));
/*
    offsetTF.translation() = Eigen::Vector3d(
            - northingReference_ - robotPose_.position.y(),
            eastingReference_ + robotPose_.position.x(),
            elevationReference_ + robotPose_.position.z());
*/
    offsetTF.translation() = Eigen::Vector3d(
            -(eastingReference_ + robotPose_.position.x()),
            -(northingReference_ + robotPose_.position.y()),
            elevationReference_ + robotPose_.position.z());
    offsetTF.linear() = offsetQuaternion.toRotationMatrix();
    pcl::transformPointCloud(*cloud_, *preprocessedCloud_, offsetTF);
/*
    const double robotYaw = robotPose_.getYaw() - M_PI / 2.;
    auto robotTF = Eigen::Affine3d::Identity();
    const auto robotQuaternion = Eigen::Quaterniond(
            Eigen::AngleAxisd(robotYaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0., Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()));

    robotTF.translation() = Eigen::Vector3d::Zero();
    robotTF.linear() = robotQuaternion.toRotationMatrix();
    pcl::transformPointCloud(*preprocessedCloud_, *preprocessedCloud_, robotTF);
*/
}

void Task::downsampleCloud(void) {
    const double voxelSize = _voxelSize.value();

    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(preprocessedCloud_);
    voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxelGrid.filter(*preprocessedCloud_);
}

void Task::cropCloud(void) {
    const Eigen::Vector4f minCutoffPoint(
            - _cropSize.rvalue() / 2.,
            - _cropSize.rvalue() / 2.,
            -std::numeric_limits<double>::max(), 0.);
    const Eigen::Vector4f maxCutoffPoint(
            _cropSize.rvalue() / 2.,
            _cropSize.rvalue() / 2.,
            std::numeric_limits<double>::max(), 0.);

    pcl::CropBox<pcl::PointXYZ> cropBox;
    cropBox.setInputCloud(preprocessedCloud_);
    cropBox.setMin(minCutoffPoint);
    cropBox.setMax(maxCutoffPoint);
    cropBox.filter(*preprocessedCloud_);
}

void Task::smoothCloud(void) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(preprocessedCloud_);
    sor.setMeanK(_sorMeanK.rvalue());
    sor.setStddevMulThresh(_sorStdMultiplier.rvalue());
    sor.filter(*preprocessedCloud_);
}

void Task::writeCloud(const Cloud::ConstPtr& cloud) {
    BaseCloud baseCloud;
    baseCloud.points.clear();
    baseCloud.points.reserve(cloud->size());
    baseCloud.time = base::Time::now();

    for (const auto& point : cloud->points)
        baseCloud.points.push_back(base::Point(point.x, point.y, point.z));

    _pointCloud.write(baseCloud);
}

}  // namespace orbiter_preprocessing


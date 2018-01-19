#pragma once

#include "orbiter_preprocessing/TaskBase.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace orbiter_preprocessing {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using BaseCloud = base::samples::Pointcloud;

class Task : public TaskBase {
  public:
    explicit Task(std::string const& name = "orbiter_preprocessing::Task");

  protected:
    bool configureHook(void) override;

    void updateHook(void) override;

    bool startHook(void) override;

    void loadCloud(void);

    void saveCloud(void);

    bool isReadyToPreprocess(void);

    void preprocessCloud(void);

    void transformCloud(void);

    void downsampleCloud(void);

    void cropCloud(void);

    void smoothCloud(void);

    void writeCloud(void);

  protected:
    base::samples::RigidBodyState robotPose_;
    base::samples::RigidBodyState lastPose_;

    Cloud::Ptr cloud_;
    Cloud::Ptr preprocessedCloud_;

    double eastingReference_;
    double northingReference_;
    double elevationReference_;
};

}  // namespace orbiter_preprocessing


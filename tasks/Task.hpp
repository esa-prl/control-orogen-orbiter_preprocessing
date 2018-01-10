#pragma once

#include "cloud_preprocessing/TaskBase.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace cloud_preprocessing {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using BaseCloud = base::samples::Pointcloud;

class Task : public TaskBase {
  public:
    explicit Task(std::string const& name = "cloud_preprocessing::Task");

  protected:
    void updateHook(void) override;

    void loadCloud(void);
    void preprocessCloud(void);
    void downsampleCloud(void);
    void transformCloud(void);
    void cropCloud(void);
    void smoothCloud(void);
    void writeCloud(void);

  protected:
    bool initialized_;

    Cloud::Ptr cloud_;
};

}  // namespace cloud_preprocessing


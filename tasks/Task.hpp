#pragma once

#include "pcl_io_preprocessing/TaskBase.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl_io_preprocessing {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using BaseCloud = base::samples::Pointcloud;

class Task : public TaskBase {
  public:
    explicit Task(std::string const& name = "pcl_io_preprocessing::Task")
            : TaskBase(name) {}

  protected:
    bool configureHook(void) override;

    void updateHook(void) override;
};

}  // namespace pcl_io_preprocessing


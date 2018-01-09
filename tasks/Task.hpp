#pragma once

#include "pcl_io_preprocessing/TaskBase.hpp"

namespace pcl_io_preprocessing {

class Task : public TaskBase {
  public:
    explicit Task(std::string const& name = "pcl_io_preprocessing::Task")
            : TaskBase(name) {}

  protected:
    bool configureHook(void) override;

    void updateHook(void) override;
};

}  // namespace pcl_io_preprocessing


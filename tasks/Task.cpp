#include "Task.hpp"

namespace pcl_io_preprocessing {

bool Task::configureHook(void) {
    if (!TaskBase::configureHook()) return false;

    return true;
}

void Task::updateHook(void) {
    TaskBase::updateHook();
}

}  // namespace pcl_io_preprocessing


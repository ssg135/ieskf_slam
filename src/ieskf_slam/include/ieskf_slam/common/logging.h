#pragma once

#include <glog/logging.h>

#define SLAM_LOG_DEBUG VLOG(1)
#define SLAM_LOG_INFO VLOG(1)
#define SLAM_LOG_WARN LOG(WARNING)
#define SLAM_LOG_ERROR LOG(ERROR)
#define SLAM_CHECK(condition) CHECK(condition)

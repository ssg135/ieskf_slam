#pragma once
#include "common_lidar_process_interface.h"
#include <cstring>
#include <limits>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace ROSNoetic
{
    class GenericPointCloud2Process : public CommonLidarProcessInterface
    {
    private:
        enum class TimeMode {
            NONE,
            RELATIVE_SECONDS,
            ABSOLUTE_SECONDS,
            OFFSET_NS,
        };

        const sensor_msgs::PointField* findField(const sensor_msgs::PointCloud2& msg,
                                                 std::initializer_list<const char*> names) const {
            for (const char* name : names) {
                const int field_index = sensor_msgs::getPointCloud2FieldIndex(msg, name);
                if (field_index >= 0) {
                    return &msg.fields[field_index];
                }
            }
            return nullptr;
        }

        template <typename T>
        T readScalar(const std::uint8_t* data_ptr) const {
            T value;
            std::memcpy(&value, data_ptr, sizeof(T));
            return value;
        }

        double readAsDouble(const std::uint8_t* point_ptr,
                            const sensor_msgs::PointField& field) const {
            const std::uint8_t* data_ptr = point_ptr + field.offset;
            switch (field.datatype) {
                case sensor_msgs::PointField::INT8:
                    return static_cast<double>(readScalar<std::int8_t>(data_ptr));
                case sensor_msgs::PointField::UINT8:
                    return static_cast<double>(readScalar<std::uint8_t>(data_ptr));
                case sensor_msgs::PointField::INT16:
                    return static_cast<double>(readScalar<std::int16_t>(data_ptr));
                case sensor_msgs::PointField::UINT16:
                    return static_cast<double>(readScalar<std::uint16_t>(data_ptr));
                case sensor_msgs::PointField::INT32:
                    return static_cast<double>(readScalar<std::int32_t>(data_ptr));
                case sensor_msgs::PointField::UINT32:
                    return static_cast<double>(readScalar<std::uint32_t>(data_ptr));
                case sensor_msgs::PointField::FLOAT32:
                    return static_cast<double>(readScalar<float>(data_ptr));
                case sensor_msgs::PointField::FLOAT64:
                    return readScalar<double>(data_ptr);
                default:
                    throw std::runtime_error("unsupported PointField datatype");
            }
        }

        TimeMode detectTimeMode(const sensor_msgs::PointCloud2& msg,
                                const sensor_msgs::PointField*& time_field) const {
            if ((time_field = findField(msg, {"offset_time"})) != nullptr) {
                return TimeMode::OFFSET_NS;
            }
            if ((time_field = findField(msg, {"timestamp"})) != nullptr) {
                return TimeMode::ABSOLUTE_SECONDS;
            }
            if ((time_field = findField(msg, {"time"})) != nullptr) {
                const double first_time = readAsDouble(msg.data.data(), *time_field);
                return first_time > 1e6 ? TimeMode::ABSOLUTE_SECONDS
                                        : TimeMode::RELATIVE_SECONDS;
            }
            time_field = nullptr;
            return TimeMode::NONE;
        }

        double readPointTimeSec(const std::uint8_t* point_ptr,
                                const sensor_msgs::PointField* time_field,
                                const TimeMode time_mode) const {
            if (time_field == nullptr || time_mode == TimeMode::NONE) {
                return 0.0;
            }
            if (time_mode == TimeMode::OFFSET_NS) {
                return readAsDouble(point_ptr, *time_field) * 1e-9;
            }
            return readAsDouble(point_ptr, *time_field);
        }

    public:
        GenericPointCloud2Process() = default;
        ~GenericPointCloud2Process() override = default;

        IESKFSLAM::PointCloud process(const sensor_msgs::PointCloud2 &msg) const override{
            IESKFSLAM::PointCloud cloud;
            cloud.cloud_ptr->clear();

            if (msg.is_bigendian) {
                ROS_ERROR_STREAM_THROTTLE(1.0, "GenericPointCloud2Process does not support big-endian PointCloud2");
                cloud.time_stamp.fromNSec(msg.header.stamp.toNSec());
                return cloud;
            }

            const sensor_msgs::PointField* x_field = findField(msg, {"x"});
            const sensor_msgs::PointField* y_field = findField(msg, {"y"});
            const sensor_msgs::PointField* z_field = findField(msg, {"z"});
            if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
                ROS_ERROR_STREAM_THROTTLE(1.0, "GenericPointCloud2Process requires x, y, z fields");
                cloud.time_stamp.fromNSec(msg.header.stamp.toNSec());
                return cloud;
            }

            const sensor_msgs::PointField* intensity_field = findField(msg, {"intensity"});
            const sensor_msgs::PointField* ring_field = findField(msg, {"ring", "line"});
            const sensor_msgs::PointField* time_field = nullptr;
            const TimeMode time_mode = detectTimeMode(msg, time_field);

            const std::size_t point_count = static_cast<std::size_t>(msg.width) *
                                            static_cast<std::size_t>(msg.height);
            if (point_count == 0 || msg.point_step == 0) {
                cloud.time_stamp.fromNSec(msg.header.stamp.toNSec());
                return cloud;
            }

            cloud.cloud_ptr->reserve(point_count);

            double min_point_time_sec = std::numeric_limits<double>::infinity();
            if (time_mode != TimeMode::NONE) {
                for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
                    const std::uint8_t* point_ptr = msg.data.data() + point_index * msg.point_step;
                    min_point_time_sec = std::min(min_point_time_sec,
                                                  readPointTimeSec(point_ptr, time_field, time_mode));
                }
            }

            if (!std::isfinite(min_point_time_sec)) {
                min_point_time_sec = 0.0;
            }

            for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
                const std::uint8_t* point_ptr = msg.data.data() + point_index * msg.point_step;
                IESKFSLAM::Point point;
                point.x = static_cast<float>(readAsDouble(point_ptr, *x_field));
                point.y = static_cast<float>(readAsDouble(point_ptr, *y_field));
                point.z = static_cast<float>(readAsDouble(point_ptr, *z_field));
                point.intensity = intensity_field == nullptr ? 0.0f
                                                             : static_cast<float>(readAsDouble(point_ptr, *intensity_field));
                point.ring = ring_field == nullptr ? 0
                                                   : static_cast<std::int32_t>(readAsDouble(point_ptr, *ring_field));

                double point_time_sec = 0.0;
                switch (time_mode) {
                    case TimeMode::OFFSET_NS:
                    case TimeMode::RELATIVE_SECONDS:
                    case TimeMode::ABSOLUTE_SECONDS:
                        point_time_sec = readPointTimeSec(point_ptr, time_field, time_mode);
                        break;
                    case TimeMode::NONE:
                        point_time_sec = 0.0;
                        break;
                }

                point.offset_time = static_cast<std::uint32_t>(
                    std::max(0.0, point_time_sec - min_point_time_sec) * 1e9);
                cloud.cloud_ptr->push_back(point);
            }

            switch (time_mode) {
                case TimeMode::ABSOLUTE_SECONDS:
                    cloud.time_stamp.fromSec(min_point_time_sec);
                    break;
                case TimeMode::RELATIVE_SECONDS:
                    cloud.time_stamp.fromSec(msg.header.stamp.toSec() + min_point_time_sec);
                    break;
                case TimeMode::OFFSET_NS:
                case TimeMode::NONE:
                    cloud.time_stamp.fromNSec(msg.header.stamp.toNSec());
                    break;
            }

            return cloud;
        }
    };
} // namespace ROSNoetic

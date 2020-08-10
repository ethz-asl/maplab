#ifndef __POINTMATCHER_ROS_POINT_CLOUD_H
#define __POINTMATCHER_ROS_POINT_CLOUD_H

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"

namespace ros
{
	struct Time;
}

namespace tf
{
	struct TransformListener;
}

namespace PointMatcher_ros
{
	template<typename T>
	typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::PointCloud2& rosMsg, const bool isDense=false);
	
	template<typename T>
	typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::LaserScan& rosMsg, const tf::TransformListener* listener = 0, const std::string& fixed_frame = "/world", const bool force3D = false, const bool addTimestamps=false, const bool addObservationDirection=false);
	
	template<typename T>
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp);
} // PointMatcher_ros

#endif //__POINTMATCHER_ROS_POINT_CLOUD_H

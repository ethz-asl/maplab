#ifndef __POINTMATCHER_ROS_TRANSFORM_H
#define __POINTMATCHER_ROS_TRANSFORM_H

#include "pointmatcher/PointMatcher.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Eigen"

namespace ros
{
	struct Time;
}

namespace tf
{
	struct Transform;
	struct TransformListener;
	struct StampedTransform;
}

namespace PointMatcher_ros
{
	// tf to Eigen
	template<typename T>
	typename PointMatcher<T>::TransformationParameters transformListenerToEigenMatrix(const tf::TransformListener &listener, const std::string& target, const std::string& source, const ros::Time& stamp);
	
	// Odom to Eigen
	template<typename T>
	typename PointMatcher<T>::TransformationParameters odomMsgToEigenMatrix(const nav_msgs::Odometry& odom);
	
	// Pose to Eigen
	template<typename T>
	typename PointMatcher<T>::TransformationParameters poseMsgToEigenMatrix(const geometry_msgs::Pose& pose);

	// Eigen to Odom
	template<typename T>
	nav_msgs::Odometry eigenMatrixToOdomMsg(const typename PointMatcher<T>::TransformationParameters& inTr, const std::string& frame_id, const ros::Time& stamp);

	// Eigen to Pose
	template<typename T>
	geometry_msgs::Pose eigenMatrixToPoseMsg(const typename PointMatcher<T>::TransformationParameters& inTr);


	// Eigen to Transform
	template<typename T>
	tf::Transform eigenMatrixToTransform(const typename PointMatcher<T>::TransformationParameters& inTr);

	// Eigen to Stamped Transform
	template<typename T>
	tf::StampedTransform eigenMatrixToStampedTransform(const typename PointMatcher<T>::TransformationParameters& inTr, const std::string& target, const std::string& source, const ros::Time& stamp);
	
	// 2D / 3D transform
	template<typename T>
	typename PointMatcher<T>::TransformationParameters eigenMatrixToDim(const typename PointMatcher<T>::TransformationParameters& matrix, int dimp1);
	
} // PointMatcher_ros

#endif //__POINTMATCHER_ROS_TRANSFORM_H

#ifndef __GET_PARAMS_FROM_SERVER_H
#define __GET_PARAMS_FROM_SERVER_H

#include <string>
#include "ros/ros.h"

bool hasParam(const std::string& name)
{
	return ros::param::has(std::string("~")+name);
}

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
	T v;
	if (ros::param::get(std::string("~")+name, v))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}

template<typename T>
T getParam(const std::string& name)
{
	T v;
	if (ros::param::get(std::string("~")+name, v))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_ERROR_STREAM("Cannot find value for parameter: " << name);
	return T();
}

#endif // __GET_PARAMS_FROM_SERVER_H
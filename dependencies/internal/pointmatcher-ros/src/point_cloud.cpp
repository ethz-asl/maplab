#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG

#include "pointmatcher_ros/point_cloud.h"
#include "ros/ros.h"
#include "boost/detail/endian.hpp"
#include "boost/algorithm/string.hpp"
//#include <boost/algorithm/string/predicate.hpp>
//#include <boost/algorithm/string/erase.hpp>
#include "tf/transform_listener.h"
#include <vector>
#include <memory>

namespace PointMatcher_ros
{
	using namespace std;
	
	//! Transform a ROS PointCloud2 message into a libpointmatcher point cloud
	template<typename T>
	typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::PointCloud2& rosMsg, const bool isDense)
	{

		//FIXME: continue from here, need to decode time properly   
		typedef PointMatcher<T> PM;
		typedef PointMatcherIO<T> PMIO;
		typedef typename PMIO::PMPropTypes PM_types;
		typedef typename PM::DataPoints DataPoints;
		typedef typename DataPoints::Label Label;
		typedef typename DataPoints::Labels Labels;
		typedef typename DataPoints::View View;
		typedef typename DataPoints::TimeView TimeView;
		
		if (rosMsg.fields.empty())
			return DataPoints();
		

		// fill labels
		// conversions of descriptor fields from pcl
		// see http://www.ros.org/wiki/pcl/Overview
		Labels featLabels;
		Labels descLabels;
		Labels timeLabels;
		vector<bool> isFeature;
		vector<PM_types> fieldTypes;
		for(auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it)
		{
			const string name(it->name);
			const size_t count(std::max<size_t>(it->count, 1));
			if (name == "x" || name == "y" || name == "z")
			{
				featLabels.push_back(Label(name, count));
				isFeature.push_back(true);
				fieldTypes.push_back(PM_types::FEATURE);
			}
			else if (name == "rgb" || name == "rgba")
			{
				descLabels.push_back(Label("color", (name == "rgba") ? 4 : 3));
				isFeature.push_back(false);
			}
			else if ((it+1) != rosMsg.fields.end() && it->name == "normal_x" && (it+1)->name == "normal_y")
			{
				if ((it+2) != rosMsg.fields.end() && (it+2)->name == "normal_z")
				{
					descLabels.push_back(Label("normals", 3));
					it += 2;
					isFeature.push_back(false);
					isFeature.push_back(false);
					fieldTypes.push_back(PM_types::DESCRIPTOR);
					fieldTypes.push_back(PM_types::DESCRIPTOR);
				}
				else
				{
					descLabels.push_back(Label("normals", 2));
					it += 1;
					isFeature.push_back(false);
					fieldTypes.push_back(PM_types::DESCRIPTOR);
				}
				isFeature.push_back(false);
				fieldTypes.push_back(PM_types::DESCRIPTOR);
			}
			else if((it+1) != rosMsg.fields.end() && boost::algorithm::ends_with(name, "_splitTime_high32") && boost::algorithm::ends_with(((it+1)->name), "_splitTime_low32"))
			{
				// time extraction
				//const string beginning = name.substr(0, name.size()-4);
				string startingName = name;
				boost::algorithm::erase_last(startingName, "_splitTime_high32");
				const string beginning = startingName;

				timeLabels.push_back(Label(beginning, 1));
				it += 1;
				isFeature.push_back(false);
				fieldTypes.push_back(PM_types::TIME);
				fieldTypes.push_back(PM_types::TIME);
			}
			//else if (name == "stamps")
			else if (name == "time")
			{
			    // timeLabels.push_back(Label(name, count));
				// fieldTypes.push_back(PM_types::TIME);
				// TODO(Perception) Check if this is the best way to treat time point descriptors. Upstream libpointmatcher's design is designed differently.
			  	descLabels.push_back(Label(name, count));
        		isFeature.push_back(false);
				fieldTypes.push_back(PM_types::DESCRIPTOR);
			}
			else
			{
				descLabels.push_back(Label(name, count));
				isFeature.push_back(false);
				fieldTypes.push_back(PM_types::DESCRIPTOR);
			}
		}

		featLabels.push_back(Label("pad", 1));
		assert(isFeature.size() == rosMsg.fields.size());
		assert(fieldTypes.size() == rosMsg.fields.size());
		
		// create cloud
		const unsigned pointCount(rosMsg.width * rosMsg.height);
		DataPoints cloud(featLabels, descLabels, timeLabels, pointCount);
		cloud.getFeatureViewByName("pad").setConstant(1);
		
		// fill cloud
		// TODO: support big endian, pass through endian-swapping method just after the *reinterpret_cast
		typedef sensor_msgs::PointField PF;
		size_t fieldId = 0;
		for(auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it, ++fieldId)
		{
			if (it->name == "rgb" || it->name == "rgba")
			{
				// special case for colors
				if (((it->datatype != PF::UINT32) && (it->datatype != PF::INT32) && (it->datatype != PF::FLOAT32)) || (it->count != 1))
					throw runtime_error(
						(boost::format("Colors in a point cloud must be a single element of size 32 bits, found %1% elements of type %2%") % it->count % unsigned(it->datatype)).str()
					);
				View view(cloud.getDescriptorViewByName("color"));
				int ptId(0);
				for (size_t y(0); y < rosMsg.height; ++y)
				{
					const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step*y);
					for (size_t x(0); x < rosMsg.width; ++x)
					{
						const uint32_t rgba(*reinterpret_cast<const uint32_t*>(dataPtr + it->offset));
						const T colorA(T((rgba >> 24) & 0xff) / 255.);
						const T colorR(T((rgba >> 16) & 0xff) / 255.);
						const T colorG(T((rgba >> 8) & 0xff) / 255.);
						const T colorB(T((rgba >> 0) & 0xff) / 255.);
						view(0, ptId) = colorR;
						view(1, ptId) = colorG;
						view(2, ptId) = colorB;
						if (view.rows() > 3)
							view(3, ptId) = colorA;
						dataPtr += rosMsg.point_step;
						ptId += 1;
					}
				}
			}
			else if(boost::algorithm::ends_with(it->name, "_splitTime_high32") || 
			        boost::algorithm::ends_with(it->name, "_splitTime_low32"))
			{
				string startingName = it->name;
				bool isHigh = false;
				if(boost::algorithm::ends_with(it->name, "_splitTime_high32"))
				{
					boost::algorithm::erase_last(startingName, "_splitTime_high32");
					isHigh = true;
				}
				if(boost::algorithm::ends_with(it->name, "_splitTime_low32"))
				{
					boost::algorithm::erase_last(startingName, "_splitTime_low32");
				}


			  TimeView timeView(cloud.getTimeViewByName(startingName));
				// use view to read data
        
				int ptId(0);
        const size_t count(std::max<size_t>(it->count, 1));
        for (size_t y(0); y < rosMsg.height; ++y)
        {
          const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step*y);
          for (size_t x(0); x < rosMsg.width; ++x)
          {
            const uint8_t* fPtr(dataPtr + it->offset);
            for (unsigned dim(0); dim < count; ++dim)
            {
							if(isHigh)
							{
								const uint32_t high32 = *reinterpret_cast<const uint32_t*>(fPtr);
								const uint32_t low32 = uint32_t(timeView(dim, ptId));
								timeView(dim, ptId) = (((uint64_t) high32) << 32) | ((uint64_t) low32);
							}
							else
							{
								const uint32_t high32 = uint32_t(timeView(dim, ptId) >> 32);
								const uint32_t low32 = *reinterpret_cast<const uint32_t*>(fPtr);
								timeView(dim, ptId) = (((uint64_t) high32) << 32) | ((uint64_t) low32);

							}
							dataPtr += rosMsg.point_step;
							ptId += 1;
						}
					}
				}

			}
			// FIXME: this might never be used
			//else if (it->name == "stamps")
			//{
			//  ROS_INFO("!!!!!!!!!!!! GOT MSG WITH STAMPS FIELD !!!!!!!!!!!!!!!!");

			//  TimeView timeView(cloud.getTimeViewByName("stamps"));

      //  // use view to read data
      //  int ptId(0);
      //  const size_t count(std::max<size_t>(it->count, 1));
      //  for (size_t y(0); y < rosMsg.height; ++y)
      //  {
      //    const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step*y);
      //    for (size_t x(0); x < rosMsg.width; ++x)
      //    {
      //      const uint8_t* fPtr(dataPtr + it->offset);
      //      for (unsigned dim(0); dim < count; ++dim)
      //      {
      //        switch (it->datatype)
      //        {
      //          case PF::INT8:
      //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const int8_t*>(fPtr));
      //            fPtr += 1;
      //            ROS_INFO("!!!!!!!!!!!! CASE INT8 !!!!!!!!!!!!!!!!");
      //            break;
      //          case PF::UINT8:
      //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const uint8_t*>(fPtr));
      //            fPtr += 1;
      //            ROS_INFO("!!!!!!!!!!!! CASE UINT8 !!!!!!!!!!!!!!!!");
      //            break;
      //          case PF::INT16:
      //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const int16_t*>(fPtr));
      //            fPtr += 2;
      //            ROS_INFO("!!!!!!!!!!!! CASE INT16 !!!!!!!!!!!!!!!!");
      //            break;
      //          case PF::UINT16:
      //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const uint16_t*>(fPtr));
      //            fPtr += 2;
      //            ROS_INFO("!!!!!!!!!!!! CASE UINT16 !!!!!!!!!!!!!!!!");
      //            break;
      //          case PF::INT32:
      //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const int32_t*>(fPtr));
      //            fPtr += 4;
      //            ROS_INFO("!!!!!!!!!!!! CASE INT32 !!!!!!!!!!!!!!!!");
      //            break;
      //          case PF::UINT32:
      //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const uint32_t*>(fPtr));
      //            fPtr += 4;
      //            ROS_INFO("!!!!!!!!!!!! CASE UINT32 !!!!!!!!!!!!!!!!");
      //            break;
      //          case PF::FLOAT32:
      //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const float*>(fPtr));
      //            fPtr += 4;
      //            //ROS_INFO("!!!!!!!!!!!! CASE FLOAT32 !!!!!!!!!!!!!!!!");
      //            break;
      //          case PF::FLOAT64:
      //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const double*>(fPtr));
      //            fPtr += 8;
      //            ROS_INFO("!!!!!!!!!!!! CASE FLOAT64 !!!!!!!!!!!!!!!!");
      //            break;
      //          default: abort();
      //        }
      //      }
      //      dataPtr += rosMsg.point_step;
      //      ptId += 1;
      //    }
      //  }
			//}
			else
			{

				// get view for editing data
				View view(
					 (it->name == "normal_x") ? cloud.getDescriptorRowViewByName("normals", 0) :
					((it->name == "normal_y") ? cloud.getDescriptorRowViewByName("normals", 1) :
					((it->name == "normal_z") ? cloud.getDescriptorRowViewByName("normals", 2) :
					((isFeature[fieldId]) ? cloud.getFeatureViewByName(it->name) :
					cloud.getDescriptorViewByName(it->name))))
				);

				// use view to read data
				int ptId(0);
				const size_t count(std::max<size_t>(it->count, 1));
				for (size_t y(0); y < rosMsg.height; ++y)
				{
					const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step*y);
					for (size_t x(0); x < rosMsg.width; ++x)
					{
						const uint8_t* fPtr(dataPtr + it->offset);
						for (unsigned dim(0); dim < count; ++dim)
						{
							switch (it->datatype)
							{
								case PF::INT8:    view(dim, ptId) = T(*reinterpret_cast<const int8_t*>(fPtr)); fPtr += 1; break;
								case PF::UINT8:   view(dim, ptId) = T(*reinterpret_cast<const uint8_t*>(fPtr)); fPtr += 1; break;
								case PF::INT16:   view(dim, ptId) = T(*reinterpret_cast<const int16_t*>(fPtr)); fPtr += 2; break;
								case PF::UINT16:  view(dim, ptId) = T(*reinterpret_cast<const uint16_t*>(fPtr)); fPtr += 2; break;
								case PF::INT32:   view(dim, ptId) = T(*reinterpret_cast<const int32_t*>(fPtr)); fPtr += 4; break;
								case PF::UINT32:  view(dim, ptId) = T(*reinterpret_cast<const uint32_t*>(fPtr)); fPtr += 4; break;
								case PF::FLOAT32: view(dim, ptId) = T(*reinterpret_cast<const float*>(fPtr)); fPtr += 4; break;
								case PF::FLOAT64: view(dim, ptId) = T(*reinterpret_cast<const double*>(fPtr)); fPtr += 8; break;
								default: abort();
							}
						}
						dataPtr += rosMsg.point_step;
						ptId += 1;
					}
				}
			}
		}

		
		if(isDense == false)
		{
			shared_ptr<typename PM::DataPointsFilter> filter(PM::get().DataPointsFilterRegistrar.create("RemoveNaNDataPointsFilter"));
			return filter->filter(cloud);
		}

		return cloud;
	}
	
	template
	PointMatcher<float>::DataPoints rosMsgToPointMatcherCloud<float>(const sensor_msgs::PointCloud2& rosMsg, const bool isDense);
	template
	PointMatcher<double>::DataPoints rosMsgToPointMatcherCloud<double>(const sensor_msgs::PointCloud2& rosMsg, const bool isDense);
	
	
	template<typename T>
	typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::LaserScan& rosMsg, const tf::TransformListener* listener, const std::string& fixedFrame, const bool force3D, const bool addTimestamps, const bool addObservationDirection)
	{
		typedef PointMatcher<T> PM;
		typedef typename PM::DataPoints DataPoints;
		typedef typename DataPoints::Label Label;
		typedef typename DataPoints::Labels Labels;
		
		Labels featLabels;
		featLabels.push_back(Label("x", 1));
		featLabels.push_back(Label("y", 1));
		if(force3D)
			featLabels.push_back(Label("z", 1));

		featLabels.push_back(Label("pad", 1));
		
    // Build descriptors
		Labels descLabels;
		if (!rosMsg.intensities.empty())
		{
			descLabels.push_back(Label("intensity", 1));
			assert(rosMsg.intensities.size() == rosMsg.ranges.size());
		}
    
    int id_obs = 0;
    if(addObservationDirection)
    {
			descLabels.push_back(Label("observationDirections", 3));
    }
		
		// Build time
		Labels timeLabels;
    if(addTimestamps)
    {
			timeLabels.push_back(Label("time", 1));
			//timeLabels.push_back(Label("stamps", 1));
    }

		// filter points based on range
    std::vector<size_t> ids(rosMsg.ranges.size());
    std::vector<double> ranges(rosMsg.ranges.size());
    std::vector<double> intensities(rosMsg.intensities.size());

		size_t goodCount(0);
		for (size_t i = 0; i < rosMsg.ranges.size(); ++i)
		{
			const float range(rosMsg.ranges[i]);
			//TODO: we might want to keep points with nan values
			if (range >= rosMsg.range_min && range <= rosMsg.range_max)
      {
        ranges[goodCount] = range;
        ids[goodCount] = i;
        if(!rosMsg.intensities.empty())
        {
          intensities[goodCount] = rosMsg.intensities[i];
        }
				++goodCount;
      }
		}
		if (goodCount == 0)
			return DataPoints();

    ids.resize(goodCount);
    ranges.resize(goodCount);
    if(!rosMsg.intensities.empty())
      intensities.resize(goodCount);

		DataPoints cloud(featLabels, descLabels, timeLabels, goodCount);
		cloud.getFeatureViewByName("pad").setConstant(1);
      
    if(addObservationDirection)
    {
      id_obs = cloud.getDescriptorStartingRow("observationDirections");
    }
		
		// fill features
		const ros::Time& startTime(rosMsg.header.stamp);
		const ros::Time endTime(startTime + ros::Duration(rosMsg.time_increment * (rosMsg.ranges.size() - 1)));
    
		for (size_t i = 0; i < ranges.size(); ++i)
		{
			const T angle = rosMsg.angle_min + ids[i]*rosMsg.angle_increment;
			const T range(ranges[i]);
      const T x = cos(angle) * range;
      const T y = sin(angle) * range;

      // the turn ratio correct for the fact that not all sensor scan
      // continuously during 360 deg 
			//TODO: check that. FP
      const float turnRatio = (rosMsg.angle_max - rosMsg.angle_min)/(2*M_PI);
      // dt_point should be more precise than rosMsg.time_increment
      const float dt_point = (rosMsg.scan_time*turnRatio)/rosMsg.ranges.size();

      if (listener)
      {
        
        const ros::Time curTime(rosMsg.header.stamp + ros::Duration(ids[i] * dt_point));

        // wait for transform
        listener->waitForTransform(
          rosMsg.header.frame_id,
          fixedFrame,
          curTime,
          ros::Duration(0.1)
        );

        // transform point
        geometry_msgs::PointStamped pin, p_out;
        pin.header.stamp = curTime;
        pin.header.frame_id = rosMsg.header.frame_id;
        pin.point.x = x;
        pin.point.y = y;
        pin.point.z = 0;
        
        // transform sensor center
        geometry_msgs::PointStamped s_in, s_out;
        s_in.header.stamp = curTime;
        s_in.header.frame_id = rosMsg.header.frame_id;
        s_in.point.x = 0;
        s_in.point.y = 0;
        s_in.point.z = 0;

        try
        {
          listener->transformPoint(
						fixedFrame,
            rosMsg.header.stamp,
            pin,
            fixedFrame,
            p_out
          );

          if(addObservationDirection)
          {
            listener->transformPoint(
                fixedFrame,
                curTime,
                s_in,
                fixedFrame,
                s_out
                );
          }
        }
        catch (const tf::ExtrapolationException& e)
        {
          //ROS_WARN_STREAM("libpointmatcher_ros: Couldn't transform point: " << e.what());
          return DataPoints();
        }

        //cout << "pin: " << pin.point.x << ", " << pin.point.y << ", " << pin.point.z << endl;
        //cout << "p_out: " << p_out.point.x << ", " << p_out.point.y << ", " << p_out.point.z << endl;

        // write back
        cloud.features(0,i) = p_out.point.x;
        cloud.features(1,i) = p_out.point.y;
        if(force3D)
          cloud.features(2,i) = p_out.point.z;
				
        if(addObservationDirection)
        {
          cloud.descriptors(id_obs  , i) = s_out.point.x - p_out.point.x;
          cloud.descriptors(id_obs+1, i) = s_out.point.y - p_out.point.y;
          cloud.descriptors(id_obs+2, i) = s_out.point.z - p_out.point.z;
        }
			}
		}

		// fill descriptors
		if (!rosMsg.intensities.empty())
		{
			auto is(cloud.getDescriptorViewByName("intensity"));
			for (size_t i = 0; i < intensities.size(); ++i)
			{
					is(0,i) = intensities[i];
			}
		}

		// Fill time
    if(addTimestamps)
    {
			//auto is(cloud.getTimeViewByName("stamps"));
			auto is(cloud.getTimeViewByName("time"));

			for (size_t i = 0; i < ranges.size(); ++i)
      {
        const ros::Time curTime(rosMsg.header.stamp + ros::Duration(ids[i] * rosMsg.time_increment));
				const boost::posix_time::time_duration duration = boost::posix_time::seconds(curTime.sec) + boost::posix_time::nanoseconds(curTime.nsec);

				is(0,i) = duration.ticks();
			}
    }

		//cerr << "point cloud:\n" << cloud.times.leftCols(10) << endl;
		return cloud;
	}
	
	template
	PointMatcher<float>::DataPoints rosMsgToPointMatcherCloud<float>(const sensor_msgs::LaserScan& rosMsg, const tf::TransformListener* listener, const std::string& fixedFrame, const bool force3D, const bool addTimestamps, const bool addObservationDirection);
	template
	PointMatcher<double>::DataPoints rosMsgToPointMatcherCloud<double>(const sensor_msgs::LaserScan& rosMsg, const tf::TransformListener* listener, const std::string& fixedFrame, const bool force3D, const bool addTimestamps, const bool addObservationDirection);


	template<typename T>
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg(const typename PointMatcher<T>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp)
	{

		sensor_msgs::PointCloud2 rosCloud;
		typedef sensor_msgs::PointField PF;
		
		// check type and get sizes
		BOOST_STATIC_ASSERT(is_floating_point<T>::value);
		BOOST_STATIC_ASSERT((is_same<T, long double>::value == false));
		uint8_t dataType;
		size_t scalarSize;
		if (typeid(T) == typeid(float))
		{
			dataType = PF::FLOAT32;
			scalarSize = 4;
		}
		else
		{
			dataType = PF::FLOAT64;
			scalarSize = 8;
		}
		
		size_t timeSize = 4; // we split in two UINT32
		
		// build labels

		// features
		unsigned offset(0);
		assert(!pmCloud.featureLabels.empty());
		assert(pmCloud.featureLabels[pmCloud.featureLabels.size()-1].text == "pad");
		for(auto it(pmCloud.featureLabels.begin()); it != pmCloud.featureLabels.end(); ++it)
		{
			// last label is padding
			if ((it + 1) == pmCloud.featureLabels.end())
				break;
			PF pointField;
			pointField.name = it->text;
			pointField.offset = offset;
			pointField.datatype= dataType;
			pointField.count = it->span;
			rosCloud.fields.push_back(pointField);
			offset += it->span * scalarSize;
		}
		bool addZ(false);
		if (!pmCloud.featureLabels.contains("z"))
		{
			PF pointField;
			pointField.name = "z";
			pointField.offset = offset;
			pointField.datatype= dataType;
			pointField.count = 1;
			rosCloud.fields.push_back(pointField);
			offset += scalarSize;
			addZ = true;
		}
		
		// descriptors
		const bool isDescriptor(!pmCloud.descriptorLabels.empty());
		bool hasColor(false);
		unsigned colorPos(0);
		unsigned colorCount(0);
		unsigned inDescriptorPos(0);
		for(auto it(pmCloud.descriptorLabels.begin()); it != pmCloud.descriptorLabels.end(); ++it)
		{
			PF pointField;
			if (it->text == "normals")
			{
				assert((it->span == 2) || (it->span == 3));
				pointField.datatype = dataType;
				pointField.name = "normal_x";
				pointField.offset = offset;
				pointField.count = 1;
				rosCloud.fields.push_back(pointField);
				offset += scalarSize;
				pointField.name = "normal_y";
				pointField.offset = offset;
				pointField.count = 1;
				rosCloud.fields.push_back(pointField);
				offset += scalarSize;
				if (it->span == 3)
				{
					pointField.name = "normal_z";
					pointField.offset = offset;
					pointField.count = 1;
					rosCloud.fields.push_back(pointField);
					offset += scalarSize;
				}
			}
			else if (it->text == "color")
			{
				colorPos = inDescriptorPos;
				colorCount = it->span;
				hasColor = true;
				pointField.datatype = (colorCount == 4) ? uint8_t(PF::UINT32) : uint8_t(PF::FLOAT32);
				pointField.name = (colorCount == 4) ? "rgba" : "rgb";
				pointField.offset = offset;
				pointField.count = 1;
				rosCloud.fields.push_back(pointField);
				offset += 4;
			}
			else
			{
				pointField.datatype = dataType;
				pointField.name = it->text;
				pointField.offset = offset;
				pointField.count = it->span;
				rosCloud.fields.push_back(pointField);
				offset += it->span * scalarSize;
			}
			inDescriptorPos += it->span;
		}

		// time
    bool hasTime(false);
    for(auto it(pmCloud.timeLabels.begin()); it != pmCloud.timeLabels.end(); ++it)
    {
      PF pointField;
      //if (it->text == "stamps")
      if (it->text == "time")
      {
        hasTime = true;

				// for Rviz view

        pointField.datatype = PF::FLOAT32;

        //pointField.datatype = PF::UINT32;
        pointField.name = "elapsedTimeSec";
        pointField.offset = offset;
        pointField.count = 1;
        rosCloud.fields.push_back(pointField);
        offset += 4;

				// Split time in two because there is not PF::UINT64
				pointField.datatype = PF::UINT32;
				pointField.name = it->text + "_splitTime_high32";
				pointField.offset = offset;
				pointField.count = 1;
				rosCloud.fields.push_back(pointField);
				offset += timeSize;
				
				pointField.datatype = PF::UINT32;
				pointField.name = it->text + "_splitTime_low32";
				pointField.offset = offset;
				pointField.count = 1;
				rosCloud.fields.push_back(pointField);
				offset += timeSize;
      }
    }

		// fill cloud with data
		rosCloud.header.frame_id = frame_id;
		rosCloud.header.stamp = stamp;
		rosCloud.height = 1;
		rosCloud.width = pmCloud.features.cols();
		#ifdef BOOST_BIG_ENDIAN
		rosCloud.is_bigendian = true;
		#else // BOOST_BIG_ENDIAN
		rosCloud.is_bigendian = false;
		#endif // BOOST_BIG_ENDIAN
		rosCloud.point_step = offset;
		rosCloud.row_step = rosCloud.point_step * rosCloud.width;
		rosCloud.is_dense = true;
		rosCloud.data.resize(rosCloud.row_step * rosCloud.height);
		
		const unsigned featureDim(pmCloud.features.rows()-1);
		const unsigned descriptorDim(pmCloud.descriptors.rows());
		const unsigned timeDim(pmCloud.times.rows());
		
		assert(descriptorDim == inDescriptorPos);
		const unsigned postColorPos(colorPos + colorCount);
		assert(postColorPos <= inDescriptorPos);
		const unsigned postColorCount(descriptorDim - postColorPos);
		
		for (unsigned pt(0); pt < rosCloud.width; ++pt)
		{
			uint8_t *fPtr(&rosCloud.data[pt * offset]);

			memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.features(0, pt)), scalarSize * featureDim);
			fPtr += scalarSize * featureDim;
			if (addZ)
			{
				memset(fPtr, 0, scalarSize);
				fPtr += scalarSize;
			}
			if (isDescriptor)
			{
				if (hasColor)
				{
					// before color
					memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(0, pt)), scalarSize * colorPos);
					fPtr += scalarSize * colorPos;
					// compact color
					uint32_t rgba;
					unsigned colorR(unsigned(pmCloud.descriptors(colorPos+0, pt) * 255.) & 0xFF);
					unsigned colorG(unsigned(pmCloud.descriptors(colorPos+1, pt) * 255.) & 0xFF);
					unsigned colorB(unsigned(pmCloud.descriptors(colorPos+2, pt) * 255.) & 0xFF);
					unsigned colorA(0);
					if (colorCount == 4)
						colorA = unsigned(pmCloud.descriptors(colorPos+3, pt) * 255.) & 0xFF;
					rgba = colorA << 24 | colorR << 16 | colorG << 8 | colorB;
					memcpy(fPtr, reinterpret_cast<const uint8_t*>(&rgba), 4);
					fPtr += 4;
					// after color
					memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(postColorPos, pt)), scalarSize * postColorCount);
					fPtr += scalarSize * postColorCount;
				}
				else
				{
					memcpy(fPtr, reinterpret_cast<const uint8_t*>(&pmCloud.descriptors(0, pt)), scalarSize * descriptorDim);
					fPtr += scalarSize * descriptorDim;
				}
			}

			// TODO: reactivate that properly
			//if(isTime)
			//{
			//	for(unsigned d = 0; d<timeDim; d++)
			//	{
			//		const uint32_t nsec = (uint32_t) pmCloud.times(d,pt);
			//		const uint32_t sec = (uint32_t) (pmCloud.times(d,pt) >> 32);
			//		memcpy(fPtr, reinterpret_cast<const uint8_t*>(&sec), timeSize);
			//		fPtr += timeSize;
			//		memcpy(fPtr, reinterpret_cast<const uint8_t*>(&nsec), timeSize);
			//		fPtr += timeSize;
			//	}
			//}
			if (hasTime) 
			{
			  // PointCloud2 can not contain uint64_t variables
			  // uint32_t are used for publishing, pmCloud.times(0, pt)/1000 (time in micro seconds)
			  
				//uint32_t temp = (uint32_t)(pmCloud.times(0, pt)/(uint64_t)1000);

				const size_t ptrSize = timeSize * timeDim;
				
				// Elapsed time
			  const float elapsedTime = (float)(pmCloud.times(0, pt) - pmCloud.times(0, 0))*1e-9f;
			  memcpy(fPtr, reinterpret_cast<const uint8_t*>(&elapsedTime), ptrSize);
				fPtr += ptrSize;
				
				// high32
				const uint32_t high32 = (uint32_t)(pmCloud.times(0, pt) >> 32);
			  memcpy(fPtr, reinterpret_cast<const uint8_t*>(&high32), ptrSize);
				fPtr += ptrSize;
				
				// low32
				const uint32_t low32 = (uint32_t)(pmCloud.times(0, pt));
			  memcpy(fPtr, reinterpret_cast<const uint8_t*>(&low32), ptrSize);
				fPtr += ptrSize;

			}
		}

		// fill remaining information
		rosCloud.header.frame_id = frame_id;
		rosCloud.header.stamp = stamp;
		
		return rosCloud;
	}
	
	template
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg<float>(const PointMatcher<float>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp);
	template
	sensor_msgs::PointCloud2 pointMatcherCloudToRosMsg<double>(const PointMatcher<double>::DataPoints& pmCloud, const std::string& frame_id, const ros::Time& stamp);

} // PointMatcher_ros

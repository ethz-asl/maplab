#ifndef __ROS_LOGGER_H
#define __ROS_LOGGER_H

#include "pointmatcher/PointMatcher.h"

namespace PointMatcherSupport
{
	struct ROSLogger: public Logger
	{
		inline static const std::string description()
		{
			return "Log using ROS console.";
		}
		
        virtual bool hasInfoChannel() const{ return true; }
		virtual void beginInfoEntry(const char *file, unsigned line, const char *func);
		virtual std::ostream* infoStream() { return &_infoStream; }
		virtual void finishInfoEntry(const char *file, unsigned line, const char *func);
		virtual bool hasWarningChannel() const { return true; }
		virtual void beginWarningEntry(const char *file, unsigned line, const char *func);
		virtual std::ostream* warningStream() { return &_warningStream; }
		virtual void finishWarningEntry(const char *file, unsigned line, const char *func);
		
	protected:
		void writeRosLog(ros::console::Level level, const char* file, int line, const char *func, const std::string& text);
		
		std::ostringstream _infoStream;
		std::ostringstream _warningStream;
	};
	
	void ROSLogger::beginInfoEntry(const char *file, unsigned line, const char *func)
	{
		_infoStream.str("");
	}
	
	void ROSLogger::finishInfoEntry(const char *file, unsigned line, const char *func)
	{
		writeRosLog(ros::console::levels::Info, file, line, func, _infoStream.str());
	}
	
	void ROSLogger::beginWarningEntry(const char *file, unsigned line, const char *func)
	{
		_warningStream.str("");
	}
	
	void ROSLogger::finishWarningEntry(const char *file, unsigned line, const char *func)
	{
		writeRosLog(ros::console::levels::Warn, file, line, func, _warningStream.str());
	}
	
	void ROSLogger::writeRosLog(ros::console::Level level, const char* file, int line, const char *func, const std::string& text)
	{
		ROSCONSOLE_DEFINE_LOCATION(true, level, ROSCONSOLE_DEFAULT_NAME);
        if (__rosconsole_define_location__enabled)
			ros::console::print(0, __rosconsole_define_location__loc.logger_, 
				__rosconsole_define_location__loc.level_, file, line, func, "%s", text.c_str());
	}
}

#endif // __ROS_LOGGER_H

// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef __POINTMATCHER_PRIVATE_H
#define __POINTMATCHER_PRIVATE_H

namespace PointMatcherSupport
{
	//! Mutex to protect creation and deletion of logger
	extern boost::mutex loggerMutex;
	//! Logger pointer
	extern std::shared_ptr<Logger> logger;
	
	// macros holding the name of current function, send patches for your favourite compiler
	#if defined(MSVC)
		#define __POINTMATCHER_FUNCTION__ __FUNCSIG__
	#elif defined(__GNUC__)
		#define __POINTMATCHER_FUNCTION__ __PRETTY_FUNCTION__
	#else
		#define __POINTMATCHER_FUNCTION__ ""
	#endif
	
	// macros for logging
	#define LOG_INFO_STREAM(args) \
	{ \
		boost::mutex::scoped_lock lock(PointMatcherSupport::loggerMutex); \
		if (PointMatcherSupport::logger.get() && \
			PointMatcherSupport::logger->hasInfoChannel()) { \
			PointMatcherSupport::logger->beginInfoEntry(__FILE__, __LINE__, __POINTMATCHER_FUNCTION__); \
			(*PointMatcherSupport::logger->infoStream()) << args; \
			PointMatcherSupport::logger->finishInfoEntry(__FILE__, __LINE__, __POINTMATCHER_FUNCTION__); \
		} \
	}
	#define LOG_WARNING_STREAM(args) \
	{ \
		boost::mutex::scoped_lock lock(PointMatcherSupport::loggerMutex); \
		if (PointMatcherSupport::logger.get() && \
			PointMatcherSupport::logger->hasWarningChannel()) { \
			PointMatcherSupport::logger->beginWarningEntry(__FILE__, __LINE__, __POINTMATCHER_FUNCTION__); \
			(*PointMatcherSupport::logger->warningStream()) << args; \
			PointMatcherSupport::logger->finishWarningEntry(__FILE__, __LINE__, __POINTMATCHER_FUNCTION__); \
		} \
	}

};

#endif // __POINTMATCHER_PRIVATE_H

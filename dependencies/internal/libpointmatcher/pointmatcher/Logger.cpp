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

#include "PointMatcher.h"
#include "PointMatcherPrivate.h"

namespace PointMatcherSupport
{
	boost::mutex loggerMutex; //!< mutex to protect access to logging 
	std::shared_ptr<Logger> logger; //!< the current logger
	
	//! Construct without parameter
	Logger::Logger()
	{}
	
	//! Construct with parameters
	Logger::Logger(const std::string& className, const ParametersDoc paramsDoc, const Parameters& params):
		Parametrizable(className,paramsDoc,params) 
	{}
	
	//! Virtual destructor, do nothing
	Logger::~Logger()
	{}
	
	//! Return whether this logger provides the info channel
	bool Logger::hasInfoChannel() const
	{
		return false;
	};
	
	//! Start a new entry into the info channel
	void Logger::beginInfoEntry(const char *file, unsigned line, const char *func)
	{}
	
	//! Return the info stream, 0 if hasInfoChannel() returns false
	std::ostream* Logger::infoStream()
	{
		return 0;
	}
	
	//! Finish the entry into the info channel
	void Logger::finishInfoEntry(const char *file, unsigned line, const char *func)
	{}
	
	//! Return whether this logger provides the warning channel
	bool Logger::hasWarningChannel() const
	{
		return false;
	}
	
	//! Start a new entry into the warning channel
	void Logger::beginWarningEntry(const char *file, unsigned line, const char *func)
	{}
	
	//! Return the warning stream, 0 if hasWarningChannel() returns false
	std::ostream* Logger::warningStream()
	{
		return 0;
	}
	
	//! Finish the entry into the warning channel
	void Logger::finishWarningEntry(const char *file, unsigned line, const char *func)
	{}
	
	//! Set a new logger, protected by a mutex
	void setLogger(std::shared_ptr<Logger> newLogger)
	{
		boost::mutex::scoped_lock lock(loggerMutex);
		logger = newLogger;
	}
}

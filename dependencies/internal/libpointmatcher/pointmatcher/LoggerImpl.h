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

#ifndef __POINTMATCHER_LOGGER_H
#define __POINTMATCHER_LOGGER_H

#include "PointMatcher.h"
#include <fstream>

namespace PointMatcherSupport
{
	struct NullLogger: public Logger
	{
		inline static const std::string description()
		{
			return "Does not log anything.";
		}
		NullLogger(): Logger("NullLogger",
												 ParametersDoc(),
												 Parameters()) {}
	};
	
	struct FileLogger: public Logger
	{
		inline static const std::string description()
		{
			return "Log using std::stream.";
		}
		inline static const ParametersDoc availableParameters()
		{
			return {
				{"infoFileName", "name of the file to output infos to, or an empty string to output infos to the standard output stream", ""},
				{"warningFileName", "name of the file to output warnings to, or an empty string to output warnings to the standard error stream", ""},
				{"displayLocation", "display the location of message in source code", "0"}
			};
		};
		
		const std::string infoFileName;
		const std::string warningFileName;
		const bool displayLocation;
		
		FileLogger(const Parameters& params = Parameters());
		
		virtual bool hasInfoChannel() const;
		virtual void beginInfoEntry(const char *file, unsigned line, const char *func);
		virtual std::ostream* infoStream();
		virtual void finishInfoEntry(const char *file, unsigned line, const char *func);
		virtual bool hasWarningChannel() const;
		virtual void beginWarningEntry(const char *file, unsigned line, const char *func);
		virtual std::ostream* warningStream();
		virtual void finishWarningEntry(const char *file, unsigned line, const char *func);
		
	protected:
		std::ofstream _infoFileStream;
		std::ofstream _warningFileStream;
		std::ostream _infoStream;
		std::ostream _warningStream;
	};
} //PointMatcherSupport

#endif // __POINTMATCHER_LOGGER_H

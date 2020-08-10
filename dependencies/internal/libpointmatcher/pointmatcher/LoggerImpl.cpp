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

#include "LoggerImpl.h"

#include <iostream>
#include <fstream>

using namespace std;

namespace PointMatcherSupport
{
	FileLogger::FileLogger(const Parameters& params):
		Logger("FileLogger", FileLogger::availableParameters(), params),
		infoFileName(Parametrizable::get<std::string>("infoFileName")),
		warningFileName(Parametrizable::get<std::string>("warningFileName")),
		displayLocation(Parametrizable::get<bool>("displayLocation")),
		_infoFileStream(infoFileName.c_str()),
		_warningFileStream(warningFileName.c_str()),
		_infoStream(nullptr),
		_warningStream(nullptr)
	{
		if (infoFileName.empty())
		{
			_infoStream.rdbuf(std::cout.rdbuf());
		}
		else
		{
			if (!_infoFileStream.good())
			{
				throw runtime_error(string("FileLogger::Cannot open info stream to file ") + infoFileName);
			}
			_infoStream.rdbuf(_infoFileStream.rdbuf());
		}

		if (warningFileName.empty())
		{
			_warningStream.rdbuf(std::cerr.rdbuf());
		}
		else
		{
			if (!_warningFileStream.good())
			{
				throw runtime_error(string("FileLogger::Cannot open warning stream to file ") + warningFileName);
			}
			_warningStream.rdbuf(_warningFileStream.rdbuf());
		}
	}
	
	bool FileLogger::hasInfoChannel() const
	{
		return true; 
	};
	
	void FileLogger::beginInfoEntry(const char *file, unsigned line, const char *func)
	{
	}
	
	std::ostream* FileLogger::infoStream()
	{
		return &_infoStream;
	}
	
	void FileLogger::finishInfoEntry(const char *file, unsigned line, const char *func)
	{
		if (displayLocation)
			_infoStream << " (at " << file << ":" << line << " in " << func << " )" << endl; 
		else
			_infoStream << endl;
	}
	
	bool FileLogger::hasWarningChannel() const
	{
		return true;
	}
	
	void FileLogger::beginWarningEntry(const char *file, unsigned line, const char *func)
	{
	}
	
	std::ostream* FileLogger::warningStream()
	{
		return &_warningStream;
	}
	
	void FileLogger::finishWarningEntry(const char *file, unsigned line, const char *func)
	{
		if (displayLocation)
			_warningStream << " (at " << file << ":" << line << " in " << func << " )" << endl;
		else
			_warningStream << endl;
	}
} //PointMatcherSupport

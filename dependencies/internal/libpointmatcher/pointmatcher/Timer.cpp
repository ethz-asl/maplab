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

#include "Timer.h"

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#ifdef _POSIX_TIMERS
namespace PointMatcherSupport
{
	timer::timer():
		_start_time(curTime())
	{
	} 
	
	void timer::restart()
	{
		_start_time = curTime();
	}
	
	double timer::elapsed() const
	{
		return  double(curTime() - _start_time) / double(1000000000);
	}
	
	timer::Time timer::curTime() const
	{
		#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
		clock_serv_t host_clock;
		mach_timespec_t now;
		host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &host_clock);
		clock_get_time(host_clock, &now);
		return Time(now.tv_sec) * Time(1000000000) + Time(now.tv_nsec);
		#else // __MACH__
		struct timespec ts;
		#ifdef CLOCK_PROCESS_CPUTIME_ID
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
		#else // BSD and old Linux
		clock_gettime(CLOCK_PROF, &ts);
		#endif
		return Time(ts.tv_sec) * Time(1000000000) + Time(ts.tv_nsec);
		#endif // __MACH__
	}
} // namespace PointMatcherSupport
#endif // _POSIX_TIMERS


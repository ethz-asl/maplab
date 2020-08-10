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

#ifndef __POINTMATCHER_TIMER_H
#define __POINTMATCHER_TIMER_H

#include <time.h>
#ifndef WIN32
#include <unistd.h>
#endif // WIN32

#ifdef _POSIX_TIMERS
namespace PointMatcherSupport
{
	/**
		High-precision timer class, using clock_gettime() or clock_get_time()
		
		The interface is a subset of the one boost::timer provides,
		but the implementation is much more precise
		on systems where clock() has low precision, such as glibc.
		
		This code gets compiled if _POSIX_TIMERS is set,
		generally in time.h or unistd.h
	*/
	struct timer
	{
		//! 64-bit time
		typedef unsigned long long Time;
		
		//! Create and start the timer
		timer();
		//! Restart the counter
		void restart();
		//! Return elapsed time in seconds
		double elapsed() const;

	private:
		//! Return time at call
		Time curTime() const;
		
		Time _start_time; //! time when counter started
	};
} // namespace PointMatcherSupport
#else // _POSIX_TIMERS
#include <boost/timer.hpp>
namespace PointMatcherSupport
{
	typedef boost::timer timer;
}
#endif // _POSIX_TIMERS

#endif // __POINTMATCHER_TIMER_H

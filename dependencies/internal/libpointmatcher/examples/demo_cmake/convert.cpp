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

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

void usage(char *argv[])
{
	cerr << "Usage " << argv[0] << " [CONFIG.yaml] INPUT.csv/.vtk/.ply OUTPUT.csv/.vtk/.ply" << endl;
	cerr << endl << "Example:" << endl;
	cerr << argv[0] << " ../examples/data/default-convert.yaml ../examples/data/cloud.00000.vtk /tmp/output.vtk" << endl << endl;
	cerr << " or " << endl;
	cerr << argv[0] << "../examples/data/cloud.00000.vtk /tmp/output.csv" << endl << endl;

}

int main(int argc, char *argv[])
{
	if (argc < 3)
	{
		usage(argv);
		return 1;
	}
	
	setLogger(PM::get().LoggerRegistrar.create("FileLogger"));

	DP d(DP::load(argv[argc-2]));
	
	if (argc == 4)
	{
		ifstream ifs(argv[1]);
		if (!ifs.good())
		{
			cerr << "Cannot open config file " << argv[1] << endl;
			usage(argv);
			return 2;
		}
		PM::DataPointsFilters f(ifs);
		f.apply(d);

	}
	
	d.save(argv[argc-1]);
	
	return 0;
}

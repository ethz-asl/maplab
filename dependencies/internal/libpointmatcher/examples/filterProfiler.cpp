/*
 * filterProfiler.cpp
 *
 *  Created on: Feb 27, 2014
 *      Author: sam
 */

#include <iostream>
#include <pointmatcher/PointMatcher.h>
#include <boost/assign.hpp>
#include <ctime>
#include <time.h>

using namespace PointMatcherSupport;
using namespace std;
using namespace boost;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

int main(int argc, char *argv[])
{

	if (argc < 2 || argc > 3)
			{
			std::cerr << "USAGE: filterProfiler <path_to_input_cloud> <summarizationMethod (optional) 2 or 1 or 0>" << std::endl;
			return -1;
			}

	char* useCentroid;
	if (argc == 3) {
			if (strcmp(argv[2],"1") != 0 && strcmp(argv[2],"0")) {
					cerr << "param useCentroid must be 1 or 0" << endl;
					return -1;
			} else
			{
					useCentroid = argv[2];
			}
	} else {
			useCentroid = "1";
	}

		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));

		DP in(DP::load(argv[1]));

		std::shared_ptr<PM::DataPointsFilter> randomSample =
			PM::get().DataPointsFilterRegistrar.create(
					"RandomSamplingDataPointsFilter",
					{{"prob", toParam(0.5)}}
			);

		cout << "starting random sample filter" << endl;
		clock_t time_a = clock();
		randomSample->inPlaceFilter(in);
		clock_t time_b = clock();

		if (time_a == ((clock_t)-1) || time_b == ((clock_t)-1))
		{
		    perror("Unable to calculate elapsed time");
		    return -1;
		}
		else
		{
		    cout << "Performed random sampling in " << (float)(time_b - time_a)/CLOCKS_PER_SEC << " seconds" << endl;
		}

		std::shared_ptr<PM::DataPointsFilter> voxelf =
			PM::get().DataPointsFilterRegistrar.create(
					"VoxelGridDataPointsFilter",
					{
						{"vSizeX", "0.2"},
						{"vSizeY", "0.2"},
						{"vSizeZ", "0.2"},
						{"useCentroid",useCentroid},
						{"averageExistingDescriptors","0"}
					}
			);

		cout << "starting voxel grid sample filter, useCentroid: " << useCentroid << endl;
		time_a = clock();
		voxelf->inPlaceFilter(in);
		time_b = clock();

		if (time_a == ((clock_t)-1) || time_b == ((clock_t)-1))
		{
			perror("Unable to calculate elapsed time");
			return -1;
		}
		else
		{
			cout << "Performed voxel grid sampling in " << (float)(time_b - time_a)/CLOCKS_PER_SEC << " seconds" << endl;
		}

		return 0;
}



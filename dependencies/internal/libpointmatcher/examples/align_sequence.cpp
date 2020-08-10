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
#include "pointmatcher/IO.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>


using namespace std;
using namespace PointMatcherSupport;

void validateArgs(int argc, char *argv[]);

/**
  * Code example for ICP taking a sequence of point clouds relatively close 
  * and build a map with them.
  * It assumes that: 3D point clouds are used, they were recorded in sequence
  * and they are express in sensor frame.
  */
int main(int argc, char *argv[])
{
	validateArgs(argc, argv);

	typedef PointMatcher<float> PM;
	typedef PointMatcherIO<float> PMIO;
	typedef PM::TransformationParameters TP;
	typedef PM::DataPoints DP;
	
	string outputFileName(argv[0]);
	
	// Rigid transformation
	std::shared_ptr<PM::Transformation> rigidTrans;
	rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

	// Create filters manually to clean the global map
	std::shared_ptr<PM::DataPointsFilter> densityFilter =
					PM::get().DataPointsFilterRegistrar.create(
						"SurfaceNormalDataPointsFilter",
						{
							{"knn", "10"},
							{"epsilon", "5"},
							{"keepNormals", "0"},
							{"keepDensities", "1"}
						}
                    );

	std::shared_ptr<PM::DataPointsFilter> maxDensitySubsample =
					PM::get().DataPointsFilterRegistrar.create(
						"MaxDensityDataPointsFilter",
						{{"maxDensity", toParam(30)}}
                    );
	// Main algorithm definition
	PM::ICP icp;

	// load YAML config
	ifstream ifs(argv[1]);
	validateFile(argv[1]);
	icp.loadFromYaml(ifs);

	PMIO::FileInfoVector list(argv[2]);

	PM::DataPoints mapPointCloud, newCloud;
	TP T_to_map_from_new = TP::Identity(4,4); // assumes 3D

	for(unsigned i=0; i < list.size(); i++)
	{
		cout << "---------------------\nLoading: " << list[i].readingFileName << endl; 

		// It is assume that the point cloud is express in sensor frame
		newCloud = DP::load(list[i].readingFileName);
		
		if(mapPointCloud.getNbPoints()  == 0)
		{
			mapPointCloud = newCloud;
			continue;
		}

		// call ICP
		try 
		{
			// We use the last transformation as a prior
			// this assumes that the point clouds were recorded in 
			// sequence. 
			const TP prior = T_to_map_from_new;

			T_to_map_from_new = icp(newCloud, mapPointCloud, prior);
		}
		catch (PM::ConvergenceError& error)
		{
			cout << "ERROR PM::ICP failed to converge: " << endl;
			cout << "   " << error.what() << endl;
			continue;
		}

		// This is not necessary in this example, but could be
		// useful if the same matrix is composed in the loop.
		T_to_map_from_new = rigidTrans->correctParameters(T_to_map_from_new);

		// Move the new point cloud in the map reference
		newCloud = rigidTrans->compute(newCloud, T_to_map_from_new);

		// Merge point clouds to map
		mapPointCloud.concatenate(newCloud);

		// Clean the map
		mapPointCloud = densityFilter->filter(mapPointCloud);
		mapPointCloud = maxDensitySubsample->filter(mapPointCloud);

		// Save the map at each iteration
		stringstream outputFileNameIter;
		outputFileNameIter << outputFileName << "_" << i << ".vtk";

		cout << "outputFileName: " << outputFileNameIter.str() << endl;
		mapPointCloud.save(outputFileNameIter.str());
	}

	return 0;
}

void validateArgs(int argc, char *argv[])
{
	if (!(argc == 3))
	{
		cerr << "Error in command line, usage " << argv[0] << " icpConfiguration.yaml listOfFiles.csv" << endl;
		cerr << endl << "Example:" << endl;
		cerr << argv[0] << " ../examples/data/default.yaml ../examples/data/carCloudList.csv" << endl;
		cerr << endl << " - or - " << endl << endl;
		cerr << argv[0] << " ../examples/data/default.yaml ../examples/data/cloudList.csv" << endl << endl;

		abort();
	}
}



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
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace PointMatcherSupport;

void validateArgs(int argc, char *argv[]);
void setupArgs(int argc, char *argv[], unsigned int& startId, unsigned int& endId, string& extension);
vector<string> loadYamlFile(string listFileName);

/**
  * Code example for DataFilter taking a sequence of point clouds with  
  * their global coordinates and build a map with a fix (manageable) number of points.
  * The example shows how to generate filters in the source code. 
  * For an example generating filters using yaml configuration, see demo_cmake/convert.cpp
  * For an example with a registration solution, see icp.cpp
  */
int main(int argc, char *argv[])
{
	validateArgs(argc, argv);

	typedef PointMatcher<float> PM;
	typedef PointMatcherIO<float> PMIO;
	typedef PM::TransformationParameters TP;
	typedef PM::DataPoints DP;

	// Process arguments
	PMIO::FileInfoVector list(argv[1]);
	const unsigned totalPointCount = boost::lexical_cast<unsigned>(argv[2]);	
	string outputFileName(argv[3]);
	
	
	setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
	
	PM::DataPoints mapCloud;

	PM::DataPoints lastCloud, newCloud;
	TP T = TP::Identity(4,4);

	// Define transformation chain
	std::shared_ptr<PM::Transformation> transformation;
	transformation = PM::get().REG(Transformation).create("RigidTransformation");

	// This filter will remove a sphere of 1 m radius. Easy way to remove the sensor self-scanning.
	std::shared_ptr<PM::DataPointsFilter> removeScanner =
		PM::get().DataPointsFilterRegistrar.create(
			"MinDistDataPointsFilter", 
			{{"minDist", "1.0"}}
		);
	
	// This filter will randomly remove 35% of the points.
	std::shared_ptr<PM::DataPointsFilter> randSubsample =
		PM::get().DataPointsFilterRegistrar.create(
			"RandomSamplingDataPointsFilter",
			{{"prob", toParam(0.65)}}
		);

	// For a complete description of filter, see 
	// https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Datafilters.md
	std::shared_ptr<PM::DataPointsFilter> normalFilter =
		PM::get().DataPointsFilterRegistrar.create(
			"SurfaceNormalDataPointsFilter",
			{
				{"knn", toParam(10)},
				{"epsilon", toParam(5)},
				{"keepNormals",toParam(1)},
				{"keepDensities",toParam(0)}
			}
		);

	std::shared_ptr<PM::DataPointsFilter> densityFilter =
		PM::get().DataPointsFilterRegistrar.create(
			"SurfaceNormalDataPointsFilter",
			{
				{"knn", "10"},
				{"epsilon", "5"},
				{"keepDensities","1"},
				{"keepNormals","0"}
			}
		);
	
	std::shared_ptr<PM::DataPointsFilter> observationDirectionFilter =
		PM::get().DataPointsFilterRegistrar.create(
			"ObservationDirectionDataPointsFilter"
		);
	
	std::shared_ptr<PM::DataPointsFilter> orientNormalFilter =
		PM::get().DataPointsFilterRegistrar.create(
			"OrientNormalsDataPointsFilter",
			{{"towardCenter", "1"}}
		);
	
	std::shared_ptr<PM::DataPointsFilter> uniformSubsample =
		PM::get().DataPointsFilterRegistrar.create(
			"MaxDensityDataPointsFilter",
			{{"maxDensity", toParam(30)}}
		);
	
	std::shared_ptr<PM::DataPointsFilter> shadowFilter =
		PM::get().DataPointsFilterRegistrar.create(
			"ShadowDataPointsFilter"
		);

	for(unsigned i=0; i < list.size(); i++)
	{
		cout << endl << "-----------------------------" << endl;
		cout << "Loading " << list[i].readingFileName;
		newCloud = DP::load(list[i].readingFileName);

		cout << " found " << newCloud.getNbPoints() << " points. " << endl;

	
		if(list[i].groundTruthTransformation.rows() != 0)
			T = list[i].groundTruthTransformation;
		else
		{
			cout << "ERROR: the field gTXX (ground truth) is required" << endl;
			abort();
		}

		PM::Parameters params;
		
		// Remove the scanner
		newCloud = removeScanner->filter(newCloud);


		// Accelerate the process and dissolve lines
		newCloud = randSubsample->filter(newCloud);
		
		// Build filter to remove shadow points and down-sample
		newCloud = normalFilter->filter(newCloud);
		newCloud = observationDirectionFilter->filter(newCloud);
		newCloud = orientNormalFilter->filter(newCloud);
		newCloud = shadowFilter->filter(newCloud);

		// Transforme pointCloud
		cout << "Transformation matrix: " << endl << T << endl;
		newCloud = transformation->compute(newCloud, T);

		if(i==0)
		{
			mapCloud = newCloud;
		}
		else
		{
			mapCloud.concatenate(newCloud);
			
			// Control point cloud size
			double probToKeep = totalPointCount/(double)mapCloud.features.cols();
			if(probToKeep < 1)
			{
				
				mapCloud = densityFilter->filter(mapCloud);
				mapCloud = uniformSubsample->filter(mapCloud);

				probToKeep = totalPointCount/(double)mapCloud.features.cols();
				
				if(probToKeep < 1)
				{
					cout << "Randomly keep " << probToKeep*100 << "\% points" << endl; 
					randSubsample = PM::get().DataPointsFilterRegistrar.create(
						"RandomSamplingDataPointsFilter", 
						{{"prob", toParam(probToKeep)}}
					);
					mapCloud = randSubsample->filter(mapCloud);
				}
			}
		}

		stringstream outputFileNameIter;
		outputFileNameIter << boost::filesystem::path(outputFileName).stem().c_str() << "_" << i << ".vtk";
		
		mapCloud.save(outputFileNameIter.str());
	}
	
	mapCloud = densityFilter->filter(mapCloud);
	mapCloud = uniformSubsample->filter(mapCloud);
	
	mapCloud = densityFilter->filter(mapCloud);

	cout << endl ;
	cout <<  "-----------------------------" << endl;
	cout <<  "-----------------------------" << endl;
	cout << "Final number of points in the map: " << mapCloud.getNbPoints() << endl;
	mapCloud.save(outputFileName);
	cout << endl ;

	return 0;
}

void validateArgs(int argc, char *argv[])
{
	if (!(argc == 4))
	{
		cerr << endl;
		cerr << "Error in command line, usage " << argv[0] << " listOfFiles.csv maxPoint outputFileName.{vtk,csv,ply}" << endl;
		cerr << endl;
		cerr << "   example using file from example/data: " << endl;
		cerr << "        " << argv[0] << " carCloudList.csv 30000 test.vtk" << endl;
		cerr << endl;
		cerr << "Note: the file listOfFiles.csv need to include a serialize transformation matrix. For example:" << endl;
		cerr << "      fileName1.vtk, T00, T01, T02, T03, T10, T11, T12, T13, T20, T21, T22, T23, T30, T31, T32" << endl;
		cerr << endl;
		cerr << "Where Txy would be a 4x4 transformation matrix:" << endl;
		cerr << "      [T00 T01 T02 T03] " << endl;
		cerr << "      [T10 T11 T12 T13] " << endl;
		cerr << "      [T20 T21 T22 T23] " << endl;
		cerr << "      [T30 T31 T32 T33] (i.e., 0,0,0,1)" << endl;
		cerr << endl;
		cerr << "For more data visit:" << endl;
		cerr << "      http://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration" << endl;
		cerr << endl;

		abort();
	}
}




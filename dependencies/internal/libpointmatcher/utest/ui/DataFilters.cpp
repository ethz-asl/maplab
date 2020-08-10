#include "../utest.h"
#include <ciso646>
#include <cmath>

using namespace std;
using namespace PointMatcherSupport;

//---------------------------
// DataFilter modules
//---------------------------

// Utility classes
class DataFilterTest: public IcpHelper
{
public:
	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for console outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
		
		// We'll test the filters on reading point cloud
		icp.readingDataPointsFilters.clear();
	}

	// Will be called for every tests
	virtual void TearDown()	{}

	void addFilter(string name, PM::Parameters params)
	{
		std::shared_ptr<PM::DataPointsFilter> testedDataPointFilter =
			PM::get().DataPointsFilterRegistrar.create(name, params);
	
		icp.readingDataPointsFilters.push_back(testedDataPointFilter);
	}
	
	void addFilter(string name)
	{
		std::shared_ptr<PM::DataPointsFilter> testedDataPointFilter =
			PM::get().DataPointsFilterRegistrar.create(name);
		
		icp.readingDataPointsFilters.push_back(testedDataPointFilter);
	}

	DP generateRandomDataPoints(int nbPoints = 100)
	{
		const int dimFeatures = 4;
		const int dimDescriptors = 3;
		const int dimTime = 2;

		PM::Matrix randFeat = PM::Matrix::Random(dimFeatures, nbPoints);
		DP::Labels featLabels;
		featLabels.push_back(DP::Label("x", 1));
		featLabels.push_back(DP::Label("y", 1));
		featLabels.push_back(DP::Label("z", 1));
		featLabels.push_back(DP::Label("pad", 1));

		PM::Matrix randDesc = PM::Matrix::Random(dimDescriptors, nbPoints);
		DP::Labels descLabels;
		descLabels.push_back(DP::Label("dummyDesc", 3));

		PM::Int64Matrix randTimes = PM::Int64Matrix::Random(dimTime, nbPoints);
		DP::Labels timeLabels;
		timeLabels.push_back(DP::Label("dummyTime", 2));

		// Construct the point cloud from the generated matrices
		DP pointCloud = DP(randFeat, featLabels, randDesc, descLabels, randTimes, timeLabels);

		return pointCloud;
	}
};

TEST_F(DataFilterTest, IdentityDataPointsFilter)
{
	// build test cloud
	DP ref2DCopy(ref2D);
	
	// apply and checked
	addFilter("IdentityDataPointsFilter");
	icp.readingDataPointsFilters.apply(ref2DCopy);
	EXPECT_TRUE(ref2D == ref2DCopy);
}

TEST_F(DataFilterTest, RemoveNaNDataPointsFilter)
{
	// build test cloud
	DP ref2DCopy(ref2D);
	int goodCount(0);
	const float nan(std::numeric_limits<float>::quiet_NaN());
	for (int i(0); i < ref2DCopy.features.cols(); ++i)
	{
		if (rand() % 3 == 0)
		{
			ref2DCopy.features(rand() % ref2DCopy.features.rows(), i) = nan;
		}
		else
			++goodCount;
	}
	
	// apply and checked
	addFilter("RemoveNaNDataPointsFilter");
	icp.readingDataPointsFilters.apply(ref2DCopy);
	EXPECT_TRUE(ref2DCopy.features.cols() == goodCount);
}

TEST_F(DataFilterTest, MaxDistDataPointsFilter)
{
	// Max dist has been selected to not affect the points
	params = PM::Parameters();
	params["dim"] = "0";
	params["maxDist"] = toParam(6.0);
	
	// Filter on x axis
	params["dim"] = "0";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Filter on y axis
	params["dim"] = "1";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Filter on z axis (not existing)
	params["dim"] = "2";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxDistDataPointsFilter", params);
	EXPECT_ANY_THROW(validate2dTransformation());
	validate3dTransformation();
	
	// Filter on a radius
	params["dim"] = "-1";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Parameter outside valid range
	params["dim"] = "3";
	//TODO: specify the exception, move that to GenericTest
	EXPECT_ANY_THROW(addFilter("MaxDistDataPointsFilter", params));
	
}

TEST_F(DataFilterTest, MinDistDataPointsFilter)
{
	// Min dist has been selected to not affect the points too much
	params = PM::Parameters();
	params["dim"] = "0";
	params["minDist"] = toParam(0.05);
	
	// Filter on x axis
	params["dim"] = "0";
	icp.readingDataPointsFilters.clear();
	addFilter("MinDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Filter on y axis
	params["dim"] = "1";
	icp.readingDataPointsFilters.clear();
	addFilter("MinDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	//TODO: move that to specific 2D test
	// Filter on z axis (not existing)
	params["dim"] = "2";
	icp.readingDataPointsFilters.clear();
	addFilter("MinDistDataPointsFilter", params);
	EXPECT_ANY_THROW(validate2dTransformation());
	validate3dTransformation();
	
	// Filter on a radius
	params["dim"] = "-1";
	icp.readingDataPointsFilters.clear();
	addFilter("MinDistDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
		
}

TEST_F(DataFilterTest, MaxQuantileOnAxisDataPointsFilter)
{
	// Ratio has been selected to not affect the points too much
	string ratio = "0.95";
	params = PM::Parameters();
	params["dim"] = "0";
	params["ratio"] = ratio;
	
	// Filter on x axis
	params["dim"] = "0";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxQuantileOnAxisDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Filter on y axis
	params["dim"] = "1";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxQuantileOnAxisDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
	
	// Filter on z axis (not existing)
	params["dim"] = "2";
	icp.readingDataPointsFilters.clear();
	addFilter("MaxQuantileOnAxisDataPointsFilter", params);
	EXPECT_ANY_THROW(validate2dTransformation());
	validate3dTransformation();
}



TEST_F(DataFilterTest, SurfaceNormalDataPointsFilter)
{
	// This filter create descriptor, so parameters should'nt impact results
	params = PM::Parameters();
	params["knn"] =  "5"; 
	params["epsilon"] =  "0.1"; 
	params["keepNormals"] =  "1";
	params["keepDensities"] =  "1";
	params["keepEigenValues"] =  "1";
	params["keepEigenVectors"] =  "1" ;
	params["keepMatchedIds"] =  "1" ;
	// FIXME: the parameter keepMatchedIds seems to do nothing...

	addFilter("SurfaceNormalDataPointsFilter", params);
	validate2dTransformation();	
	validate3dTransformation();

	// TODO: standardize how filter are tested:
	// 1- impact on number of points
	// 2- impact on descriptors
	// 3- impact on ICP (that's what we test now)
}

TEST_F(DataFilterTest, MaxDensityDataPointsFilter)
{
	// Ratio has been selected to not affect the points too much
 	vector<double> ratio = {100, 1000, 5000};
 
 	for(unsigned i=0; i < ratio.size(); i++)
 	{
 		icp.readingDataPointsFilters.clear();
		params = PM::Parameters();
		params["knn"] = "5"; 
		params["epsilon"] = "0.1"; 
		params["keepNormals"] = "0";
		params["keepDensities"] = "1";
		params["keepEigenValues"] = "0";
		params["keepEigenVectors"] = "0" ;
		params["keepMatchedIds"] = "0" ;

		addFilter("SurfaceNormalDataPointsFilter", params);

 		params = PM::Parameters();
		params["maxDensity"] = toParam(ratio[i]);

 		addFilter("MaxDensityDataPointsFilter", params);
 		
		// FIXME BUG: the density in 2D is not well computed
		//validate2dTransformation();	
 
 		//double nbInitPts = data2D.features.cols();
 		//double nbRemainingPts = icp.getPrefilteredReadingPtsCount();
 		//EXPECT_TRUE(nbRemainingPts < nbInitPts);
 		
 		validate3dTransformation();

		double nbInitPts = data3D.features.cols();
 		double nbRemainingPts = icp.getPrefilteredReadingPtsCount();
 		EXPECT_TRUE(nbRemainingPts < nbInitPts);
 	}
}

TEST_F(DataFilterTest, SamplingSurfaceNormalDataPointsFilter)
{
	// This filter create descriptor AND subsample
	params = PM::Parameters();
	params["knn"] = "5";
	params["averageExistingDescriptors"] = "1";
	params["keepNormals"] = "1";
	params["keepDensities"] = "1";
	params["keepEigenValues"] = "1";
	params["keepEigenVectors"] = "1";
	
	addFilter("SamplingSurfaceNormalDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();

}

//TODO: this filter is broken, fix it!
/*
TEST_F(DataFilterTest, ElipsoidsDataPointsFilter)
{
	// This filter creates descriptor AND subsamples
	params = PM::Parameters();
	params["knn"] = "5";
	params["averageExistingDescriptors"] = "1";
	params["keepNormals"] = "1";
	params["keepDensities"] = "1";
	params["keepEigenValues"] = "1";
	params["keepEigenVectors"] = "1";
	params["maxBoxDim"] = "inf";
	params["maxTimeWindow"] = "10";
	params["minPlanarity"] = "0";
	params["keepMeans"] = "1";
	params["keepCovariances"] = "1";
	params["keepWeights"] = "1";
	params["keepShapes"] = "1";
	params["keepIndices"] = "1";

	addFilter("ElipsoidsDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
}
*/

TEST_F(DataFilterTest, GestaltDataPointsFilter)
{
	// This filter creates descriptor AND subsamples
	params = PM::Parameters();
	params["knn"] = "5";
	params["averageExistingDescriptors"] = "1";
	params["keepNormals"] = "1";
	params["keepEigenValues"] = "1";
	params["keepEigenVectors"] = "1";
	params["maxBoxDim"] = "1";
	params["keepMeans"] = "1";
	params["keepCovariances"] = "1";
	params["keepGestaltFeatures"] = "1";
	params["radius"] = "1";
	params["ratio"] = "0.5";

	addFilter("GestaltDataPointsFilter", params);
	validate3dTransformation();
}

TEST_F(DataFilterTest, OrientNormalsDataPointsFilter)
{
	// Used to create normal for reading point cloud
	std::shared_ptr<PM::DataPointsFilter> extraDataPointFilter;
	extraDataPointFilter = PM::get().DataPointsFilterRegistrar.create(
			"SurfaceNormalDataPointsFilter");
	icp.readingDataPointsFilters.push_back(extraDataPointFilter);
	addFilter("ObservationDirectionDataPointsFilter");
	addFilter("OrientNormalsDataPointsFilter", {
		{"towardCenter", toParam(false)}
	}
	);
	validate2dTransformation();
	validate3dTransformation();
}


TEST_F(DataFilterTest, RandomSamplingDataPointsFilter)
{
	vector<double> prob = {0.80, 0.85, 0.90, 0.95};
	for(unsigned i = 0; i < prob.size(); i++)
	{
		// Try to avoid to low value for the reduction to avoid under sampling
		params = PM::Parameters();
		params["prob"] = toParam(prob[i]);

		icp.readingDataPointsFilters.clear();
		addFilter("RandomSamplingDataPointsFilter", params);
		validate2dTransformation();
		validate3dTransformation();
	}
}

TEST_F(DataFilterTest, FixStepSamplingDataPointsFilter)
{
	vector<unsigned> steps = {1, 2, 3};
	for(unsigned i=0; i<steps.size(); i++)
	{
		// Try to avoid too low value for the reduction to avoid under sampling
		params = PM::Parameters();
		params["startStep"] = toParam(steps[i]);

		icp.readingDataPointsFilters.clear();
		addFilter("FixStepSamplingDataPointsFilter", params);
		validate2dTransformation();
		validate3dTransformation();
	}
}

TEST_F(DataFilterTest, MaxPointCountDataPointsFilter)
{
	DP cloud = ref3D;
	
	const size_t maxCount = 1000;
		
	params = PM::Parameters(); 
	params["seed"] = "42";
	params["maxCount"] = toParam(maxCount);
	
	std::shared_ptr<PM::DataPointsFilter> maxPtsFilter =
			PM::get().DataPointsFilterRegistrar.create("MaxPointCountDataPointsFilter", params);

	DP filteredCloud = maxPtsFilter->filter(cloud);
	
	//Check number of points
	EXPECT_GT(cloud.getNbPoints(), filteredCloud.getNbPoints());
	EXPECT_EQ(cloud.getDescriptorDim(), filteredCloud.getDescriptorDim());
	EXPECT_EQ(cloud.getTimeDim(), filteredCloud.getTimeDim());
	
	EXPECT_EQ(filteredCloud.getNbPoints(), maxCount);
	
	//Same seed should result same filtered cloud
	DP filteredCloud2 = maxPtsFilter->filter(cloud);
	
	EXPECT_TRUE(filteredCloud == filteredCloud2);
	
	//Different seeds should not result same filtered cloud but same number
	params.clear();
	params["seed"] = "1";
	params["maxCount"] = toParam(maxCount);
	
	std::shared_ptr<PM::DataPointsFilter> maxPtsFilter2 =
			PM::get().DataPointsFilterRegistrar.create("MaxPointCountDataPointsFilter", params);
			
	DP filteredCloud3 = maxPtsFilter2->filter(cloud);
	
	EXPECT_FALSE(filteredCloud3 == filteredCloud2);
	
	EXPECT_EQ(filteredCloud3.getNbPoints(), maxCount);
	
	EXPECT_EQ(filteredCloud3.getNbPoints(), filteredCloud2.getNbPoints());
	EXPECT_EQ(filteredCloud3.getDescriptorDim(), filteredCloud2.getDescriptorDim());
	EXPECT_EQ(filteredCloud3.getTimeDim(), filteredCloud2.getTimeDim());
	
	//Validate transformation
	icp.readingDataPointsFilters.clear();
	addFilter("MaxPointCountDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
}

TEST_F(DataFilterTest, OctreeGridDataPointsFilter)
{
	const unsigned int nbPts = 60000;
	const DP cloud = generateRandomDataPoints(nbPts);	
	params = PM::Parameters(); 

	std::shared_ptr<PM::DataPointsFilter> octreeFilter;
	
	for(const int meth : {0,1,2,3})
		for(const size_t maxData : {1,5})
			for(const float maxSize : {0.,0.05})
			{
				params.clear();
				params["maxPointByNode"] = toParam(maxData);
				params["maxSizeByNode"] = toParam(maxSize);
				params["samplingMethod"] = toParam(meth);
				params["buildParallel"] = "1";
	
				octreeFilter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", params);

				const DP filteredCloud = octreeFilter->filter(cloud);
			
				if(maxData==1 and maxSize==0.)
				{
					// 1/pts by octants + validate parallel build
					// the number of point should not change
					// the sampling methods should not change anything
					//Check number of points
					EXPECT_EQ(cloud.getNbPoints(), filteredCloud.getNbPoints());
					EXPECT_EQ(cloud.getDescriptorDim(), filteredCloud.getDescriptorDim());
					EXPECT_EQ(cloud.getTimeDim(), filteredCloud.getTimeDim());
	
					EXPECT_EQ(filteredCloud.getNbPoints(), nbPts);
				}
				else
				{	
					//Check number of points
					EXPECT_GT(cloud.getNbPoints(), filteredCloud.getNbPoints());
				}
				//Validate transformation
				icp.readingDataPointsFilters.clear();
				addFilter("OctreeGridDataPointsFilter", params);
				validate2dTransformation();
				validate3dTransformation();
			}
}

TEST_F(DataFilterTest, NormalSpaceDataPointsFilter)
{
	const size_t nbPts = 60000;
	DP cloud = generateRandomDataPoints(nbPts);	
	params = PM::Parameters(); 
	
	//const size_t nbPts2D = ref2D.getNbPoints();
	const size_t nbPts3D = ref3D.getNbPoints();
	
	std::shared_ptr<PM::DataPointsFilter> nssFilter;
	
	//Compute normals
	auto paramsNorm = PM::Parameters();
			paramsNorm["knn"] = "5"; 
			paramsNorm["epsilon"] = "0.1"; 
			paramsNorm["keepNormals"] = "1";
	std::shared_ptr<PM::DataPointsFilter> normalFilter = PM::get().DataPointsFilterRegistrar.create("SurfaceNormalDataPointsFilter", paramsNorm);

	normalFilter->inPlaceFilter(cloud);
	
	//Evaluate filter
	std::vector<size_t> samples = {/* 2*nbPts2D/3, nbPts2D,*/ 1500, 5000, nbPts, nbPts3D};
	for(const float epsilon : {M_PI/6., M_PI/32., M_PI/64.})
		for(const size_t nbSample : samples)
		{
			icp.readingDataPointsFilters.clear();
			
			params.clear();
			params["epsilon"] = toParam(epsilon);
			params["nbSample"] = toParam(nbSample);

			nssFilter = PM::get().DataPointsFilterRegistrar.create("NormalSpaceDataPointsFilter", params);

			addFilter("SurfaceNormalDataPointsFilter", paramsNorm);
			addFilter("NormalSpaceDataPointsFilter", params);
			
			const DP filteredCloud = nssFilter->filter(cloud);
					
			/*
			if(nbSample <= nbPts2D)
			{
				validate2dTransformation();
				EXPECT_LE(filteredCloud.getNbPoints(), nbPts2D);
				continue;
			}
			else if (nbSample == nbPts3D)
			{
				EXPECT_EQ(filteredCloud.getNbPoints(), nbPts3D);
			}
			else */ 
			if (nbSample == nbPts)
			{
				//Check number of points
				EXPECT_EQ(cloud.getNbPoints(), filteredCloud.getNbPoints());
				EXPECT_EQ(cloud.getDescriptorDim(), filteredCloud.getDescriptorDim());
				EXPECT_EQ(cloud.getTimeDim(), filteredCloud.getTimeDim());

				EXPECT_EQ(filteredCloud.getNbPoints(), nbPts);
			}
			
			validate3dTransformation();			
			EXPECT_GE(cloud.getNbPoints(), filteredCloud.getNbPoints());
		}
}

TEST_F(DataFilterTest, CovarianceSamplingDataPointsFilter)
{
	const size_t nbPts = 60000;
	DP cloud = generateRandomDataPoints(nbPts);	
	params = PM::Parameters(); 
	
	const size_t nbPts3D = ref3D.getNbPoints();
	
	std::shared_ptr<PM::DataPointsFilter> covsFilter;
	
	//Compute normals
	auto paramsNorm = PM::Parameters();
			paramsNorm["knn"] = "5"; 
			paramsNorm["epsilon"] = "0.1"; 
			paramsNorm["keepNormals"] = "1";
	std::shared_ptr<PM::DataPointsFilter> normalFilter = PM::get().DataPointsFilterRegistrar.create("SurfaceNormalDataPointsFilter", paramsNorm);

	normalFilter->inPlaceFilter(cloud);
	
	//Evaluate filter
	std::vector<size_t> samples = {500, 1500, 5000, nbPts, nbPts3D};
	for(const size_t nbSample : samples)
	{
		icp.readingDataPointsFilters.clear();
		
		params.clear();
		params["nbSample"] = toParam(nbSample);

		covsFilter = PM::get().DataPointsFilterRegistrar.create("CovarianceSamplingDataPointsFilter", params);

		addFilter("SurfaceNormalDataPointsFilter", paramsNorm);
		addFilter("CovarianceSamplingDataPointsFilter", params);
		
		const DP filteredCloud = covsFilter->filter(cloud);
				
		if (nbSample == nbPts3D)
		{
			EXPECT_EQ(filteredCloud.getNbPoints(), nbPts3D);
		}
		else if (nbSample == nbPts)
		{
			//Check number of points
			EXPECT_EQ(cloud.getNbPoints(), filteredCloud.getNbPoints());
			EXPECT_EQ(cloud.getDescriptorDim(), filteredCloud.getDescriptorDim());
			EXPECT_EQ(cloud.getTimeDim(), filteredCloud.getTimeDim());

			EXPECT_EQ(filteredCloud.getNbPoints(), nbPts);
		}
		
		validate3dTransformation();			
		EXPECT_GE(cloud.getNbPoints(), filteredCloud.getNbPoints());
	}
}

TEST_F(DataFilterTest, VoxelGridDataPointsFilter)
{
	// Test with point cloud
	DP cloud = generateRandomDataPoints();

	params = PM::Parameters(); 
	params["vSizeX"] = "0.5";
	params["vSizeY"] = "0.5";
	params["vSizeZ"] = "0.5";
	params["useCentroid"] = toParam(true);
	params["averageExistingDescriptors"] = toParam(true);

	std::shared_ptr<PM::DataPointsFilter> voxelFilter =
			PM::get().DataPointsFilterRegistrar.create("VoxelGridDataPointsFilter", params);

	DP filteredCloud = voxelFilter->filter(cloud);

	EXPECT_GT(cloud.getNbPoints(), filteredCloud.getNbPoints());
	EXPECT_EQ(cloud.getDescriptorDim(), filteredCloud.getDescriptorDim());
	EXPECT_EQ(cloud.getTimeDim(), filteredCloud.getTimeDim());

	// Test with ICP
	vector<bool> useCentroid = {false, true};
	vector<bool> averageExistingDescriptors = {false, true};
	for (unsigned i = 0 ; i < useCentroid.size() ; i++) 
	{
		for (unsigned j = 0; j < averageExistingDescriptors.size(); j++) 
		{
			params = PM::Parameters(); 
			params["vSizeX"] = "0.02";
			params["vSizeY"] = "0.02";
			params["vSizeZ"] = "0.02";
			params["useCentroid"] = toParam(true);
			params["averageExistingDescriptors"] = toParam(true);
			
			icp.readingDataPointsFilters.clear();
			addFilter("VoxelGridDataPointsFilter", params);
			validate2dTransformation();
		}
	}

	for (unsigned i = 0 ; i < useCentroid.size() ; i++)
	{
		for (unsigned j = 0; j < averageExistingDescriptors.size(); j++)
		{
			params = PM::Parameters();
			params["vSizeX"] = "1";
			params["vSizeY"] = "1";
			params["vSizeZ"] = "1";
			params["useCentroid"] = toParam(true);
			params["averageExistingDescriptors"] = toParam(true);
			
			icp.readingDataPointsFilters.clear();
			addFilter("VoxelGridDataPointsFilter", params);
			validate3dTransformation();
		}
	}
}

TEST_F(DataFilterTest, CutAtDescriptorThresholdDataPointsFilter)
{
	// Copied from density ratio above
	vector<double> thresholds = {100, 1000, 5000};

	DP ref3Ddensities = ref3D;
	// Adding descriptor "densities"
	icp.readingDataPointsFilters.clear();
	params = PM::Parameters();
	params["knn"] = "5"; 
	params["epsilon"] = "0.1"; 
	params["keepNormals"] = "0";
	params["keepDensities"] = "1";
	params["keepEigenValues"] = "0";
	params["keepEigenVectors"] = "0";
	params["keepMatchedIds"] = "0";
	

	addFilter("SurfaceNormalDataPointsFilter", params);
	icp.readingDataPointsFilters.apply(ref3Ddensities);

	for(unsigned i=0; i < thresholds.size(); i++)
	{
		int belowCount=0;
		int aboveCount=0;

		// counting points above and below
		PM::DataPoints::View densities = ref3Ddensities.getDescriptorViewByName("densities");
		for (unsigned j=0; j < densities.cols(); ++j)
		{
			if (densities(0, j) <= thresholds[i])
			{
				++belowCount;
			}
			if (densities(0, j) >= thresholds[i])
			{
				++aboveCount;
			}
		}

		for(bool useLargerThan(true); useLargerThan; useLargerThan=false)
		{
			DP ref3DCopy = ref3Ddensities;

			icp.readingDataPointsFilters.clear();
			params = PM::Parameters(); 
			params["descName"] = toParam("densities");
			params["useLargerThan"] = toParam(useLargerThan);
			params["threshold"] = toParam(thresholds[i]);
			

			addFilter("CutAtDescriptorThresholdDataPointsFilter", params);
			icp.readingDataPointsFilters.apply(ref3DCopy);
			if (useLargerThan)
			{
				EXPECT_TRUE(ref3DCopy.features.cols() == belowCount);
			}
			else
			{
				EXPECT_TRUE(ref3DCopy.features.cols() == aboveCount);
			}
		}
	}
}

TEST_F(DataFilterTest, DistanceLimitDataPointsFilter)
{
	params = PM::Parameters();
	params["dim"] = "0";
	params["dist"] = toParam(6.0);
	params["removeInside"] = "0";

	// Filter on x axis
	params["dim"] = "0";
	icp.readingDataPointsFilters.clear();
	addFilter("DistanceLimitDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();

	// Filter on y axis
	params["dim"] = "1";
	icp.readingDataPointsFilters.clear();
	addFilter("DistanceLimitDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();

	// Filter on z axis (not existing)
	params["dim"] = "2";
	icp.readingDataPointsFilters.clear();
	addFilter("DistanceLimitDataPointsFilter", params);
	EXPECT_ANY_THROW(validate2dTransformation());
	validate3dTransformation();

	// Filter on a radius
	params["dim"] = "-1";
	icp.readingDataPointsFilters.clear();
	addFilter("DistanceLimitDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();

	// Parameter outside valid range
	params["dim"] = "3";
	//TODO: specify the exception, move that to GenericTest
	EXPECT_ANY_THROW(addFilter("DistanceLimitDataPointsFilter", params));


	params = PM::Parameters();
	params["dim"] = "0";
	params["dist"] = toParam(0.05);
	params["removeInside"] = "1";

	// Filter on x axis
	params["dim"] = "0";
	icp.readingDataPointsFilters.clear();
	addFilter("DistanceLimitDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();

	// Filter on y axis
	params["dim"] = "1";
	icp.readingDataPointsFilters.clear();
	addFilter("DistanceLimitDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();

	//TODO: move that to specific 2D test
	// Filter on z axis (not existing)
	params["dim"] = "2";
	icp.readingDataPointsFilters.clear();
	addFilter("DistanceLimitDataPointsFilter", params);
	EXPECT_ANY_THROW(validate2dTransformation());
	validate3dTransformation();

	// Filter on a radius
	params["dim"] = "-1";
	icp.readingDataPointsFilters.clear();
	addFilter("DistanceLimitDataPointsFilter", params);
	validate2dTransformation();
	validate3dTransformation();
}

TEST_F(DataFilterTest, SameFilterInstanceTwice)
{
	params = PM::Parameters();
	
	std::shared_ptr<PM::DataPointsFilter> df = PM::get().DataPointsFilterRegistrar.create("MaxPointCountDataPointsFilter", params);
	
	icp.referenceDataPointsFilters.push_back(df);
	icp.readingDataPointsFilters.push_back(df);
}

TEST_F(DataFilterTest, RemoveSensorBiasDataPointsFilter)
{
	const size_t nbPts = 6;
	const double expectedErrors_LMS1xx[6] = {0., -0.0015156, -0.059276,
						 0., -0.002311, -0.163689};
	const double expectedErrors_HDL32E[6]  = {0., -0.002945, -0.075866,
						  0., -0.002998,-0.082777 };
	
	PM::Matrix points = (PM::Matrix(3,6) << 1, 1, 1, 5, 5, 5,
		0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0).finished();
	DP::Labels pointsLabels;
	pointsLabels.push_back(DP::Label("x", 1));
	pointsLabels.push_back(DP::Label("y", 1));
	pointsLabels.push_back(DP::Label("z", 1));
	pointsLabels.push_back(DP::Label("pad", 1));
	
	PM::Matrix desc = (PM::Matrix(4,6) << 0., 0.7854, 1.4835, 0., 0.7854, 1.4835, //0,45,85 degrees
		-1, -1, -1, -5, -5, -5,
		0,  0,  0,  0,  0,  0, // observation : point to sensor
		0,  0,  0,  0,  0,  0).finished();
	
	PM::Matrix desc2 = (PM::Matrix(4,6) << 0., 0.7854, std::nanf(""), 0., 0.7854, M_PI_2, //0,45,90 degrees
		-1, -1, -1, -5, -5, -5,
		0,  0,  0,  0,  0,  0, // observation : point to sensor
		0,  0,  0,  0,  0,  0).finished();
	DP::Labels descLabels;
	descLabels.push_back(DP::Label("incidenceAngles", 1));
	descLabels.push_back(DP::Label("observationDirections", 3));
	
	PM::Int64Matrix randTimes = PM::Int64Matrix::Random(2, nbPts);
	DP::Labels timeLabels;
	timeLabels.push_back(DP::Label("dummyTime", 2));
	
	// Construct the point cloud from the generated matrices
	DP pointCloud = DP(points, pointsLabels, desc, descLabels, randTimes, timeLabels);
	
	
	PM::Parameters parameters;
	parameters["sensorType"] = toParam(0); //LMS_1xx
	std::shared_ptr<PM::DataPointsFilter> removeSensorBiasFilter = PM::get().DataPointsFilterRegistrar.create("RemoveSensorBiasDataPointsFilter", parameters);
	DP resultCloud = removeSensorBiasFilter->filter(pointCloud);
	EXPECT_EQ(pointCloud.getNbPoints(), resultCloud.getNbPoints());
	
	for(std::size_t i = 0; i< nbPts; ++i)
	{
		const double error = pointCloud.features.col(i).norm() - resultCloud.features.col(i).norm();
		EXPECT_NEAR(expectedErrors_LMS1xx[i], error, 1e-3); // below mm
	}
	
	parameters["sensorType"] = toParam(1); //HDL32E
	removeSensorBiasFilter = PM::get().DataPointsFilterRegistrar.create("RemoveSensorBiasDataPointsFilter", parameters);
	resultCloud = removeSensorBiasFilter->filter(pointCloud);
	
	for(std::size_t i = 0; i< nbPts; ++i)
	{
		const double error = pointCloud.features.col(i).norm() - resultCloud.features.col(i).norm();
		EXPECT_NEAR(expectedErrors_HDL32E[i], error, 1e-4); // below mm
	}


	//test points rejection
	pointCloud = DP(points, pointsLabels, desc2, descLabels, randTimes, timeLabels);

	parameters["sensorType"] = toParam(0); //LMS_1xx
	parameters["angleThreshold"] = toParam(30.);
	removeSensorBiasFilter = PM::get().DataPointsFilterRegistrar.create("RemoveSensorBiasDataPointsFilter", parameters);
	resultCloud = removeSensorBiasFilter->filter(pointCloud);	

	//four points should have been rejected
	EXPECT_EQ(pointCloud.getNbPoints()-4, resultCloud.getNbPoints());
}

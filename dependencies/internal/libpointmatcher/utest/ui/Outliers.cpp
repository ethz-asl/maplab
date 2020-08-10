#include "../utest.h"
#include "pointmatcher/OutlierFiltersImpl.h"

using namespace std;
using namespace PointMatcherSupport;

//---------------------------
// Outlier modules
//---------------------------

// Utility classes
class OutlierFilterTest: public IcpHelper
{
public:
	std::shared_ptr<PM::OutlierFilter> testedOutlierFilter;

	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for console outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
		
		icp.outlierFilters.clear();
	}

	// Will be called for every tests
	virtual void TearDown(){}

	void addFilter(string name, PM::Parameters params)
	{
		testedOutlierFilter = 
			PM::get().OutlierFilterRegistrar.create(name, params);
		icp.outlierFilters.push_back(testedOutlierFilter);
	}

};

//No commun parameters were found for 2D and 3D, tests are spliced
TEST_F(OutlierFilterTest, MaxDistOutlierFilter2D)
{
	addFilter("MaxDistOutlierFilter", {
			{"maxDist", toParam(0.10)}
		}
	);
	validate2dTransformation();
}

TEST_F(OutlierFilterTest, MaxDistOutlierFilter3D)
{
	addFilter("MaxDistOutlierFilter", {
			{"maxDist", toParam(1.0)}
		}
	);
	validate3dTransformation();
}

//No commun parameters were found for 2D and 3D, tests are spliced
TEST_F(OutlierFilterTest, MinDistOutlierFilter2D)
{
	// Since not sure how useful is that filter, we keep the 
	// MaxDistOutlierFilter with it
	std::shared_ptr<PM::OutlierFilter> extraOutlierFilter;
	
	extraOutlierFilter = 
		PM::get().OutlierFilterRegistrar.create(
			"MaxDistOutlierFilter", {
				{"maxDist", toParam(0.10)}
			}
		);
	icp.outlierFilters.push_back(extraOutlierFilter);
	
	addFilter("MinDistOutlierFilter", {{"minDist", toParam(0.0002)}});
	
	validate2dTransformation();
}

TEST_F(OutlierFilterTest, MinDistOutlierFilter3D)
{
	// Since not sure how useful is that filter, we keep the 
	// MaxDistOutlierFilter with it
	std::shared_ptr<PM::OutlierFilter> extraOutlierFilter;
	
	extraOutlierFilter = 
		PM::get().OutlierFilterRegistrar.create(
			"MaxDistOutlierFilter", {
				{"maxDist", toParam(1.0)}
			}
		)
	;
	icp.outlierFilters.push_back(extraOutlierFilter);
	
	addFilter("MinDistOutlierFilter", {{"minDist", toParam(0.0002)}});
	
	validate3dTransformation();
}

TEST_F(OutlierFilterTest, MedianDistOutlierFilter)
{
	addFilter("MedianDistOutlierFilter", {{"factor", toParam(3.5)}});
	validate2dTransformation();
	validate3dTransformation();
}


TEST_F(OutlierFilterTest, TrimmedDistOutlierFilter)
{
	addFilter("TrimmedDistOutlierFilter", {{"ratio", toParam(0.85)}});
	validate2dTransformation();
	validate3dTransformation();
}


TEST_F(OutlierFilterTest, VarTrimmedDistOutlierFilter)
{
	addFilter("VarTrimmedDistOutlierFilter", {
			{"minRatio", toParam(0.60)},
			{"maxRatio", toParam(0.80)},
			{"lambda", toParam(0.9)}
		}
	);
	validate2dTransformation();
	validate3dTransformation();
}

OutlierFiltersImpl<float>::OutlierWeights VarTrimLambdaTest(const float lambda) {
	OutlierFiltersImpl<float>::VarTrimmedDistOutlierFilter filter({{"minRatio", toParam(0.0000001)},
																   {"maxRatio", toParam(1.0)},
																   {"lambda", toParam(lambda)}});
	PointMatcher<float>::DataPoints filteredReading;
	PointMatcher<float>::DataPoints filteredReference;

	// Create a vector a distance
	PointMatcher<float>::Matches::Dists dists(1, 5);
	dists << 4, 5, 5, 5, 5;
	PointMatcher<float>::Matches::Ids ids(1, 5);
	PointMatcher<float>::Matches input(dists, ids);
	return filter.compute(filteredReading, filteredReference, input);
}

TEST_F(OutlierFilterTest, VarTrimmedDistOutlierFilterParameters)
{
	// A lambda parameter of zero, all matches will be reject except for the minimum
	OutlierFiltersImpl<float>::OutlierWeights weights = VarTrimLambdaTest(0.0);
	// The minimum is the first value
	ASSERT_EQ(1.0f, weights(0, 0));
	ASSERT_EQ(0.0f, weights(0, 1));

	weights = VarTrimLambdaTest(1.0);
	ASSERT_EQ(1.0f, weights(0, 0));
	ASSERT_EQ(1.0f, weights(0, 1));
}

#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

//---------------------------
// Error modules
//---------------------------

// Utility classes
class ErrorMinimizerTest: public IcpHelper
{
public:
	std::shared_ptr<PM::ErrorMinimizer> errorMin;

	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for consol outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
	}

	// Will be called for every tests
	virtual void TearDown(){}

	void setError(string name)
	{
		errorMin = PM::get().ErrorMinimizerRegistrar.create(name);
		icp.errorMinimizer = errorMin;
	}
};


TEST_F(ErrorMinimizerTest, PointToPointErrorMinimizer)
{
	setError("PointToPointErrorMinimizer");
	validate2dTransformation();
	validate3dTransformation();
}

TEST_F(ErrorMinimizerTest, PointToPlaneErrorMinimizer)
{
	setError("PointToPlaneErrorMinimizer");
	validate2dTransformation();
	validate3dTransformation();
}

TEST_F(ErrorMinimizerTest, ErrorElements)
{
	const unsigned int nbPoints = 100;
	const unsigned int dimFeatures = 4;
	const unsigned int dimDescriptors = 3;
	const unsigned int dimTime = 2;

	// Fake DataPoints
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
	DP request = DP(randFeat, featLabels, randDesc, descLabels, randTimes, timeLabels);
	DP source =  DP(randFeat, featLabels, randDesc, descLabels, randTimes, timeLabels);
	
	// Fake Weights
	PM::OutlierWeights weights = PM::OutlierWeights::Ones(1, nbPoints);

	// Fake Matches
	PM::Matches::Ids ids(1, nbPoints);
	PM::Matches::Dists dists(1, nbPoints);

	for(unsigned int i=0; i<nbPoints; i++)
	{
		ids(0,i) = i;
		dists(0,i) = 1.0;
	}

	// Construct matches from generated matrices
	PM::Matches matches(dists, ids);

	
	// Finally, the constructor for ErrorElements
	PM::ErrorMinimizer::ErrorElements mPts(request, source, weights, matches);

	// check number of points
	EXPECT_EQ(mPts.reference.getNbPoints(), nbPoints);
	EXPECT_EQ(mPts.reference.getNbPoints(), mPts.reading.getNbPoints());

	// check descriptor
	EXPECT_EQ(mPts.reference.getDescriptorDim(), dimDescriptors);
	
	// check time
	EXPECT_EQ(mPts.reference.getTimeDim(), dimTime);

}


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

#include "utest.h"

using namespace std;
using namespace PointMatcherSupport;

// TODO: avoid global by using testing::Environment

std::string dataPath;

DP ref2D;
DP data2D;
DP ref3D;
DP data3D;
PM::TransformationParameters validT2d;
PM::TransformationParameters validT3d;

//---------------------------
// Test ICP with all existing filters.

// Algorithm:
// 1. Iterate over all yaml files in
//    libpointmatcher/examples/data/icp_data, each file tests ICP
//    with one or more filters.
// 2. Run ICP with the given yaml file. The filters in the yaml
//    file are applied along the way.
// 3. Write the obtained ICP transform to disk, to the same directory,
//    with file extension .cur_trans (for easy future comparisons).
// 4. Load the reference (known as correct) ICP transform from disk,
//    from the same directory, with file extension .ref_trans.
// 5. See if the current and reference transforms are equal.

// To update an existing test or add a new test, simply add/modify
// the desired yaml file, run the tests (they may fail this time), then
// copy the (just written) desired current transform file on top of the
// corresponding reference transform file. Run the tests again. This
// time they will succeed.
//---------------------------

// Find the median coefficient of a matrix
double median_coeff(Eigen::MatrixXf& A){
  Eigen::Map<Eigen::VectorXf> v(A.data(),A.size());
  std::sort(v.data(), v.data() + v.size());
  return v[v.size()/2];
}

TEST(icpTest, icpTest)
{
	DP ref  = DP::load(dataPath + "cloud.00000.vtk");
	DP data = DP::load(dataPath + "cloud.00001.vtk");

	namespace fs = boost::filesystem;
	fs::path config_dir(dataPath + "icp_data");
	EXPECT_TRUE( fs::exists(config_dir) && fs::is_directory(config_dir) );

	fs::directory_iterator end_iter;
	for( fs::directory_iterator d(config_dir); d != end_iter; ++d)
	{
		if (!fs::is_regular_file(d->status()) ) continue;

		std::cout << "Testing file " << d->path().string() << std::endl;
		// Load config file, and form ICP object
		PM::ICP icp;
		std::string config_file = d->path().string();
		if (fs::extension(config_file) != ".yaml") continue;
		std::ifstream ifs(config_file.c_str());
		EXPECT_NO_THROW(icp.loadFromYaml(ifs)) << "This error was caused by the test file:" << endl << "   " << config_file;

		// Compute current ICP transform
		PM::TransformationParameters curT = icp(data, ref);

		// Write current transform to disk (to easily compare it
		// with reference transform offline)
		fs::path cur_file = d->path();
		cur_file.replace_extension(".cur_trans");
		//std::cout << "Writing: " << cur_file << std::endl;
		std::ofstream otfs(cur_file.c_str());
		otfs.precision(16);
		otfs << curT;
		otfs.close();
                
		// Load reference transform
		fs::path ref_file = d->path();
		ref_file.replace_extension(".ref_trans");
		PM::TransformationParameters refT = 0*curT;
		//std::cout << "Reading: " << ref_file << std::endl;
		std::ifstream itfs(ref_file.c_str());
		EXPECT_TRUE(itfs.good()) << "Could not find " << ref_file
					 << ". If this is the first time this test is run, "
					 << "create it as a copy of " << cur_file; 
		
		for (int row = 0; row < refT.cols(); row++)
		{
			for (int col = 0; col < refT.cols(); col++)
			{
				itfs >>refT(row, col);
			}
		}

		// Dump the reference transform and current one
		//std::cout.precision(17);
		//std::cout << "refT:\n" << refT << std::endl;
		//std::cout << "curT:\n" << curT << std::endl;

		// We need to compare the stored icp transform vs the computed one.
		// Since the icp solution is not unique, they may differ a lot.
		// Yet, the point of icp is
		// curT*data = ref, and refT*data = ref
		// so no matter what, the difference curT*data - refT*data
		// must be small, which is what we will test for.

		// Find the median absolute difference between curT*data and refT*data
		Eigen::MatrixXf AbsDiff = (curT*data.features - refT*data.features).array().abs();
		double median_diff = median_coeff(AbsDiff);

		// Find the median absolute value of curT*data
		Eigen::MatrixXf Data = (curT*data.features).array().abs();
		double median_data = median_coeff(Data);

		// Find the relative error
		double rel_err = median_diff/median_data;

		// A relative error of 3% is probably acceptable. 
		EXPECT_LT(rel_err, 0.03) << "This error was caused by the test file:" << endl << "   " << config_file;
	}
}

TEST(icpTest, icpSingular)
{
	// Here we test point-to-plane ICP where the point clouds underdetermine transformation
	// This situation requires special treatment in the algorithm.

	// create a x-y- planar grid point cloud in points
	const size_t nX = 10, nY = nX;
	Eigen::MatrixXf points(4, nX * nY);
	const float d = 0.1;
	const float oX = -(nX * d / 2), oY = -(nY * d / 2);

	for(size_t x = 0; x < nX; x++){
		for(size_t y = 0; y < nY; y++){
			points.col( x * nY + y) << d * x + oX, d * y + oY, 0, 1;
		}
	}

	DP pts0;
	pts0.features = points;
	DP pts1;
	points.row(2).setOnes();
	pts1.features = points; // pts1 is pts0 shifted by one in z-direction

	PM::ICP icp;
	std::string config_file = dataPath + "default-identity.yaml";
	EXPECT_TRUE(boost::filesystem::exists(config_file));

	std::ifstream ifs(config_file.c_str());
	EXPECT_NO_THROW(icp.loadFromYaml(ifs)) << "This error was caused by the test file:" << endl << "   " << config_file;

	// Compute ICP transform
	PM::TransformationParameters curT = icp(pts0, pts1);

	PM::Matrix expectedT = PM::Matrix::Identity(4,4);
	expectedT(2,3) = 1;
	EXPECT_TRUE(expectedT.isApprox(curT)) << "Expecting pure translation in z-direction of unit distance." << endl;
}

TEST(icpTest, icpIdentity)
{
	// Here we test point-to-plane ICP where we expect the output transform to be 
	// the identity. This situation requires special treatment in the algorithm.
	const float epsilon = 0.0001;
	
	DP pts0 = DP::load(dataPath + "cloud.00000.vtk");
	DP pts1 = DP::load(dataPath + "cloud.00000.vtk");

	PM::ICP icp;
	std::string config_file = dataPath + "default-identity.yaml";
	EXPECT_TRUE(boost::filesystem::exists(config_file));

	std::ifstream ifs(config_file.c_str());
	EXPECT_NO_THROW(icp.loadFromYaml(ifs)) << "This error was caused by the test file:" << endl << "   " << config_file;

	// Compute current ICP transform
	PM::TransformationParameters curT = icp(pts0, pts1);

	EXPECT_TRUE(curT.isApprox(PM::Matrix::Identity(4, 4), epsilon)) << "Expecting identity transform." << endl;
}

TEST(icpTest, similarityTransform)
{
	// Here we test similarity point-to-point ICP.
	
	DP pts0 = DP::load(dataPath + "car_cloud400.csv");
	DP pts1 = DP::load(dataPath + "car_cloud400_scaled.csv");

	PM::ICP icp;
	std::string config_file = dataPath + "icp_data/defaultSimilarityPointToPointMinDistDataPointsFilter.yaml";
	EXPECT_TRUE(boost::filesystem::exists(config_file));

	std::ifstream ifs(config_file.c_str());
	EXPECT_NO_THROW(icp.loadFromYaml(ifs)) << "This error was caused by the test file:" << endl << "   " << config_file;

	// Compute current ICP transform
	PM::TransformationParameters curT = icp(pts0, pts1);

	// We know the scale we're looking for is 1.04.
	double scale = pow(curT.determinant(), 1.0/3.0);
	EXPECT_LT( std::abs(scale - 1.04), 0.001)
	  << "Expecting the similarity transform scale to be 1.04.";
}

TEST(icpTest, icpSequenceTest)
{
	DP pts0 = DP::load(dataPath + "cloud.00000.vtk");
	DP pts1 = DP::load(dataPath + "cloud.00001.vtk");
	DP pts2 = DP::load(dataPath + "cloud.00002.vtk");
	
	PM::TransformationParameters Ticp = PM::Matrix::Identity(4,4);

	PM::ICPSequence icpSequence;

	std::ifstream ifs((dataPath + "default.yaml").c_str());
	icpSequence.loadFromYaml(ifs);

	EXPECT_FALSE(icpSequence.hasMap());

	DP map = icpSequence.getPrefilteredInternalMap();
	EXPECT_EQ(map.getNbPoints(), 0u);
	EXPECT_EQ(map.getHomogeneousDim(), 0u);
	
	map = icpSequence.getPrefilteredMap();
	EXPECT_EQ(map.getNbPoints(), 0u);
	EXPECT_EQ(map.getHomogeneousDim(), 0u);

	icpSequence.setMap(pts0);
	map = icpSequence.getPrefilteredInternalMap();
	EXPECT_LE(map.getNbPoints(), pts0.getNbPoints());
	EXPECT_GT(map.getNbPoints(), 0u);
	EXPECT_EQ(map.getHomogeneousDim(), pts0.getHomogeneousDim());

	Ticp = icpSequence(pts1);
	map = icpSequence.getPrefilteredMap();
	EXPECT_LE(map.getNbPoints(), pts0.getNbPoints());
	EXPECT_GT(map.getNbPoints(), 0u);
	EXPECT_EQ(map.getHomogeneousDim(), pts0.getHomogeneousDim());
	
	Ticp = icpSequence(pts2);
	map = icpSequence.getPrefilteredMap();
	EXPECT_LE(map.getNbPoints(), pts0.getNbPoints());
	EXPECT_GT(map.getNbPoints(), 0u);
	EXPECT_EQ(map.getHomogeneousDim(), pts0.getHomogeneousDim());

	icpSequence.clearMap();
	map = icpSequence.getPrefilteredInternalMap();
	EXPECT_EQ(map.getNbPoints(), 0u);
	EXPECT_EQ(map.getHomogeneousDim(), 0u);
}

// Utility classes
class GenericTest: public IcpHelper
{

public:

	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for consol outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
	}

	// Will be called for every tests
	virtual void TearDown()
	{	
	}
};

//---------------------------
// Generic tests
//---------------------------

TEST_F(GenericTest, ICP_default)
{
	validate2dTransformation();
	validate3dTransformation();
}

//---------------------------
// Main
//---------------------------
int main(int argc, char **argv)
{
	dataPath = "";
	for(int i=1; i < argc; i++)
	{
		if (strcmp(argv[i], "--path") == 0 && i+1 < argc)
			dataPath = argv[i+1];
	}

	if(dataPath == "")
	{
		cerr << "Missing the flag --path ./path/to/examples/data\n Please give the path to the test data folder which should be included with the source code. The folder is named 'examples/data'." << endl;
		return -1;
	}

	// Load point cloud for all test
	ref2D =  DP::load(dataPath + "2D_oneBox.csv");
	data2D = DP::load(dataPath + "2D_twoBoxes.csv");
	ref3D =  DP::load(dataPath + "car_cloud400.csv");
	data3D = DP::load(dataPath + "car_cloud401.csv");

	// Result of data express in ref (from visual inspection)
	validT2d = PM::TransformationParameters(3,3);
	validT2d <<  0.987498,  0.157629, 0.0859918,
				-0.157629,  0.987498,  0.203247,
						0,         0,         1;

	validT3d = PM::TransformationParameters(4,4);
	validT3d <<   0.982304,   0.166685,  -0.0854066,  0.0446816,
	 			 -0.150189,   0.973488,   0.172524,   0.191998,
	   			  0.111899,  -0.156644,   0.981296,  -0.0356313,
	              0,          0,          0,          1;

	testing::GTEST_FLAG(print_time) = true;
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}




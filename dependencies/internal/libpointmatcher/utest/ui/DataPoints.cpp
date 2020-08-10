#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

//---------------------------
// Point-cloud structures
//---------------------------

TEST(PointCloudTest, CopyConstructor2D)
{
	const DP ref2DCopy(ref2D);
	EXPECT_TRUE(ref2DCopy.features == ref2D.features);
	EXPECT_TRUE(ref2DCopy.featureLabels == ref2D.featureLabels);
	EXPECT_TRUE(ref2DCopy.descriptors == ref2D.descriptors);
	EXPECT_TRUE(ref2DCopy.descriptorLabels == ref2D.descriptorLabels);
	EXPECT_TRUE(ref2DCopy == ref2D);
}


TEST(PointCloudTest, FeatureConstructor2D)
{
	const DP ref2DCopy(ref2D.features, ref2D.featureLabels);
	EXPECT_TRUE(ref2DCopy.features == ref2D.features);
	EXPECT_TRUE(ref2DCopy.featureLabels == ref2D.featureLabels);
	EXPECT_TRUE(ref2DCopy == ref2D);
	EXPECT_TRUE(ref2DCopy.descriptors.rows() == 0);
	EXPECT_TRUE(ref2DCopy.descriptors.cols() == 0);
}

TEST(PointCloudTest, FeatureConstructor3D)
{
	// Note: this test cover also the operator ==

	////// 1-Empty constructor
	DP ref3DCopy = DP();
	EXPECT_TRUE(ref3DCopy.features.rows() == 0);
	EXPECT_TRUE(ref3DCopy.features.cols() == 0);
	EXPECT_TRUE(ref3DCopy.descriptors.rows() == 0);
	EXPECT_TRUE(ref3DCopy.descriptors.cols() == 0);
	EXPECT_TRUE(ref3DCopy.times.rows() == 0);
	EXPECT_TRUE(ref3DCopy.times.cols() == 0);


	////// 2-Constructor with only features
	ref3DCopy = DP(ref3D.features, ref3D.featureLabels);
	EXPECT_TRUE(ref3DCopy.features == ref3D.features);
	EXPECT_TRUE(ref3DCopy.featureLabels == ref3D.featureLabels);
	
	// descriptor missing in ref3DCopy
	EXPECT_FALSE(ref3DCopy == ref3D); 
	
	EXPECT_TRUE(ref3DCopy.descriptors.rows() == 0);
	EXPECT_TRUE(ref3DCopy.descriptors.cols() == 0);

	////// 3-Constructor with features and descriptors
	ref3DCopy = DP(ref3D.features, ref3D.featureLabels, ref3D.descriptors, ref3D.descriptorLabels);
	
	EXPECT_TRUE(ref3DCopy.features == ref3D.features);
	EXPECT_TRUE(ref3DCopy.featureLabels == ref3D.featureLabels);
	EXPECT_TRUE(ref3DCopy.descriptors== ref3D.descriptors);
	EXPECT_TRUE(ref3DCopy.descriptorLabels == ref3D.descriptorLabels);
	

	EXPECT_TRUE(ref3DCopy == ref3D); 
	
	////// 4-Copy constructor
	ref3DCopy = DP(ref3D);
	
	EXPECT_TRUE(ref3DCopy.features == ref3D.features);
	EXPECT_TRUE(ref3DCopy.featureLabels == ref3D.featureLabels);
	EXPECT_TRUE(ref3DCopy.descriptors== ref3D.descriptors);
	EXPECT_TRUE(ref3DCopy.descriptorLabels == ref3D.descriptorLabels);
	

	EXPECT_TRUE(ref3DCopy == ref3D);
}


TEST(PointCloudTest, ConstructorWithData)
{
	const int nbPoints = 100;
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
	
	EXPECT_EQ(pointCloud.features.rows(), dimFeatures);
	EXPECT_EQ(pointCloud.features.cols(), nbPoints);
	EXPECT_EQ(pointCloud.descriptors.rows(), dimDescriptors);
	EXPECT_EQ(pointCloud.descriptors.cols(), nbPoints);
	EXPECT_EQ(pointCloud.times.rows(), dimTime);
	EXPECT_EQ(pointCloud.times.cols(), nbPoints);



}

TEST(PointCloudTest, ConcatenateFeatures2D)
{
	const int leftPoints(ref2D.features.cols() / 2);
	const int rightPoints(ref2D.features.cols() - leftPoints);
	DP lefts(
		ref2D.features.leftCols(leftPoints),
		ref2D.featureLabels
	);
	DP rights(
		ref2D.features.rightCols(rightPoints),
		ref2D.featureLabels
	);
	lefts.concatenate(rights);
	EXPECT_TRUE(lefts == ref2D);
}

TEST(PointCloudTest, ConcatenateFeatures3D)
{
	const int leftPoints(ref3D.features.cols() / 2);
	const int rightPoints(ref3D.features.cols() - leftPoints);
	DP lefts(
		ref3D.features.leftCols(leftPoints),
		ref3D.featureLabels
	);
	DP rights(
		ref3D.features.rightCols(rightPoints),
		ref3D.featureLabels
	);
	lefts.concatenate(rights);
	EXPECT_TRUE(lefts.features == ref3D.features);
}

TEST(PointCloudTest, ConcatenateDescSame)
{
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	
	const int leftPoints(ref2D.features.cols() / 2);
	const int rightPoints(ref2D.features.cols() - leftPoints);
	DP lefts(
		ref2D.features.leftCols(leftPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(5, leftPoints),
		Labels(Label("Desc5D", 5))
	);
	DP rights(
		ref2D.features.rightCols(rightPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(5, rightPoints),
		Labels(Label("Desc5D", 5))
	);
	lefts.concatenate(rights);
	EXPECT_TRUE(lefts.descriptors.rows() == 5);
	EXPECT_TRUE(lefts.descriptors.cols() == lefts.features.cols());
}

TEST(PointCloudTest, ConcatenateDescSame2)
{
	typedef DP::Label Label;
	
	DP ref3DCopy(ref3D.features, ref3D.featureLabels);
	ref3DCopy.descriptorLabels.push_back(Label("Desc5D", 5));
	ref3DCopy.descriptors = PM::Matrix::Random(5, ref3DCopy.features.cols());
	
	const int leftPoints(ref3DCopy.features.cols() / 2);
	const int rightPoints(ref3DCopy.features.cols() - leftPoints);
	DP lefts(
		ref3DCopy.features.leftCols(leftPoints),
		ref3DCopy.featureLabels,
		ref3DCopy.descriptors.leftCols(leftPoints),
		ref3DCopy.descriptorLabels
	);
	DP rights(
		ref3DCopy.features.rightCols(rightPoints),
		ref3DCopy.featureLabels,
		ref3DCopy.descriptors.rightCols(rightPoints),
		ref3DCopy.descriptorLabels
	);
	lefts.concatenate(rights);
	EXPECT_TRUE(lefts == ref3DCopy);
}

TEST(PointCloudTest, ConcatenateDescDiffName)
{
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	
	const int leftPoints(ref2D.features.cols() / 2);
	const int rightPoints(ref2D.features.cols() - leftPoints);
	DP lefts(
		ref2D.features.leftCols(leftPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(5, leftPoints),
		Labels(Label("MyDesc5D", 5))
	);
	DP rights(
		ref2D.features.rightCols(rightPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(5, rightPoints),
		Labels(Label("YourDesc5D", 5))
	);
	lefts.concatenate(rights);
	EXPECT_TRUE(lefts.descriptors.rows() == 0);
	EXPECT_TRUE(lefts.descriptors.cols() == 0);
}

TEST(PointCloudTest, ConcatenateDescDiffSize)
{
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	
	const int leftPoints(ref2D.features.cols() / 2);
	const int rightPoints(ref2D.features.cols() - leftPoints);
	DP lefts(
		ref2D.features.leftCols(leftPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(3, leftPoints),
		Labels(Label("DescND", 3))
	);
	DP rights(
		ref2D.features.rightCols(rightPoints),
		ref2D.featureLabels,
		PM::Matrix::Random(5, rightPoints),
		Labels(Label("DescND", 5))
	);
	EXPECT_THROW(lefts.concatenate(rights), DP::InvalidField);
}

TEST(PointCloudTest, AssertConsistency)
{
	DP ref2DCopy(ref2D);

  // Point cloud is in order after loading it
  EXPECT_NO_THROW(ref2DCopy.assertDescriptorConsistency());

  // We add only a descriptor label without descriptor
  PM::DataPoints::Labels labels;
  labels.push_back(PM::DataPoints::Label("FakeDesc", 2));
  ref2DCopy.descriptorLabels = labels;
	EXPECT_THROW(ref2DCopy.assertDescriptorConsistency(), std::runtime_error);
 

  // We add a descriptor with the wrong number of points
  PM::Matrix descriptors = PM::Matrix::Random(2, 10);
  ref2DCopy.descriptors = descriptors;
	
  EXPECT_THROW(ref2DCopy.assertDescriptorConsistency(), std::runtime_error);
  
  // We add a descriptor with the wrong dimension
  descriptors = PM::Matrix::Random(1, ref2DCopy.getNbPoints());
  ref2DCopy.descriptors = descriptors;
  EXPECT_THROW(ref2DCopy.assertDescriptorConsistency(), std::runtime_error);

}

TEST(PointCloudTest, GetInfo)
{
	//cerr << ref2D.features.rows() << endl;
	//cerr << ref2D.features.cols() << endl;
	//cerr << ref2D.descriptors.rows() << endl;
	//cerr << ref2D.descriptors.cols() << endl;
	
	EXPECT_EQ(ref3D.getNbPoints(), 24989u);
	EXPECT_EQ(ref3D.getEuclideanDim(), 3u);
	EXPECT_EQ(ref3D.getHomogeneousDim(), 4u);
	EXPECT_EQ(ref3D.getNbGroupedDescriptors(), 1u);
	EXPECT_EQ(ref3D.getDescriptorDim(), 3u);
	
	EXPECT_EQ(ref2D.getNbPoints(), 361u);
	EXPECT_EQ(ref2D.getEuclideanDim(), 2u);
	EXPECT_EQ(ref2D.getHomogeneousDim(), 3u);
	EXPECT_EQ(ref2D.getNbGroupedDescriptors(), 0u);
	EXPECT_EQ(ref2D.getDescriptorDim(), 0u);

}

TEST(PointCloudTest, AddRemove)
{
	DP ref3DCopy = ref3D;
	const int testedValue = 9;

	//////Add features
	PM::Matrix newFeature = PM::Matrix::Ones(1,ref3DCopy.getNbPoints())*testedValue;
	ref3DCopy.addFeature("testF", newFeature);

	//Is the new row added?
	EXPECT_EQ(ref3DCopy.getHomogeneousDim(), ref3D.getHomogeneousDim()+1);
	
	//Is padding still at the end?
	EXPECT_EQ(ref3DCopy.featureLabels.back().text, "pad");

	//Is the value right?
	DP::View newFeatureView = ref3DCopy.getFeatureViewByName("testF");
	EXPECT_EQ(newFeatureView(0,0), testedValue);

	//////Remove features
	ref3DCopy.removeFeature("testF");

	// Is the extra data removed?
	EXPECT_TRUE(ref3DCopy.features.isApprox(ref3D.features));


	//////Add descriptors
	const int testedValue2 = 88;
	PM::Matrix newDescriptor4D = PM::Matrix::Ones(4,ref3DCopy.getNbPoints())*testedValue;
	PM::Matrix newDescriptor2D = PM::Matrix::Ones(2,ref3DCopy.getNbPoints())*testedValue2;

	ref3DCopy.addDescriptor("test4D", newDescriptor4D);
	ref3DCopy.addDescriptor("test2D", newDescriptor2D);
	
	//Is the new row added?
	EXPECT_EQ(ref3DCopy.getDescriptorDim(), ref3D.getDescriptorDim()+6);
	EXPECT_EQ(ref3DCopy.getNbGroupedDescriptors(), ref3D.getNbGroupedDescriptors()+2);

	//Is the value right?
	DP::View newDescriptor4DView = ref3DCopy.getDescriptorViewByName("test4D");
	EXPECT_EQ(newDescriptor4DView(0,0), testedValue);
	DP::View newDescriptor2DView = ref3DCopy.getDescriptorViewByName("test2D");
	EXPECT_EQ(newDescriptor2DView(0,0), testedValue2);


	//////Remove descriptors
	ref3DCopy.removeDescriptor("test4D");
	ref3DCopy.removeDescriptor("test2D");
	
	//removing random name shoudn't have any effect
	ref3DCopy.removeDescriptor("grrrrr");

	// Is the extra data removed?
	EXPECT_TRUE(ref3DCopy.descriptors.isApprox(ref3D.descriptors));

}

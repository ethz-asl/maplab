#ifndef UTEST_HPP_
#define UTEST_HPP_

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"
#include "../contrib/gtest/gtest.h"

#include <string>
#include <fstream>

#include "boost/filesystem.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

extern std::string dataPath;

extern DP ref2D;
extern DP data2D;
extern DP ref3D;
extern DP data3D;
extern PM::TransformationParameters validT2d;
extern PM::TransformationParameters validT3d;

//---------------------------
// Base for ICP tests
//---------------------------

class IcpHelper: public testing::Test
{
public:
	
	PM::ICP icp;
	
	PM::Parameters params;

	virtual void dumpVTK()
	{
		// Make available a VTK inspector for manual inspection
		icp.inspector =
			PM::get().InspectorRegistrar.create(
				"VTKFileInspector", 
				{{"baseFileName", "./unitTest"}}
			);
	}
	
	void validate2dTransformation()
	{
		const PM::TransformationParameters testT = icp(data2D, ref2D);
		const int dim = validT2d.cols();

		const BOOST_AUTO(validTrans, validT2d.block(0, dim-1, dim-1, 1).norm());
		const BOOST_AUTO(testTrans, testT.block(0, dim-1, dim-1, 1).norm());
	
		const BOOST_AUTO(validAngle, acos(validT2d(0,0)));
		const BOOST_AUTO(testAngle, acos(testT(0,0)));
		
		EXPECT_NEAR(validTrans, testTrans, 0.05);
		EXPECT_NEAR(validAngle, testAngle, 0.05);
	}

	void validate3dTransformation()
	{
		//dumpVTK();

		const PM::TransformationParameters testT = icp(data3D, ref3D);
		const int dim = validT2d.cols();

		const BOOST_AUTO(validTrans, validT3d.block(0, dim-1, dim-1, 1).norm());
		const BOOST_AUTO(testTrans, testT.block(0, dim-1, dim-1, 1).norm());
	
		const BOOST_AUTO(testRotation, Eigen::Quaternion<float>(Eigen::Matrix<float,3,3>(testT.topLeftCorner(3,3))));
		const BOOST_AUTO(validRotation, Eigen::Quaternion<float>(Eigen::Matrix<float,3,3>(validT3d.topLeftCorner(3,3))));
		
		const BOOST_AUTO(angleDist, validRotation.angularDistance(testRotation));
		
		//cout << testT << endl;
		//cout << "angleDist: " << angleDist << endl;
		//cout << "transDist: " << abs(validTrans-testTrans) << endl;
		EXPECT_NEAR(validTrans, testTrans, 0.1);
		EXPECT_NEAR(angleDist, 0.0, 0.1);

	}
};

#endif /* UTEST_HPP_ */



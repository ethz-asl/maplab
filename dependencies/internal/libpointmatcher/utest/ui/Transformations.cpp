#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;


//---------------------------
// Transformation Checker modules
//---------------------------

// Utility classes
class TransformationCheckerTest: public IcpHelper
{
public:
	std::shared_ptr<PM::TransformationChecker> transformCheck;

	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for consol outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
		
		icp.transformationCheckers.clear();
	}

	// Will be called for every tests
	virtual void TearDown(){}

	void addFilter(string name, PM::Parameters params)
	{
		transformCheck = 
			PM::get().TransformationCheckerRegistrar.create(name, params);
		
		icp.transformationCheckers.push_back(transformCheck);
	}
};


TEST_F(TransformationCheckerTest, CounterTransformationChecker)
{
	addFilter("CounterTransformationChecker", {{"maxIterationCount", toParam(20)}});
	validate2dTransformation();
}

TEST_F(TransformationCheckerTest, DifferentialTransformationChecker)
{
	addFilter("DifferentialTransformationChecker", {
			{"minDiffRotErr", toParam(0.001)},
			{"minDiffTransErr", toParam(0.001)},
			{"smoothLength", toParam(4)}
		}
	);
	validate2dTransformation();
}

TEST_F(TransformationCheckerTest, BoundTransformationChecker)
{
	// Since that transChecker is trigger when the distance is growing
	// and that we do not expect that to happen in the test dataset, we
	// keep the Counter to get out of the looop	
	std::shared_ptr<PM::TransformationChecker> extraTransformCheck;
	
	extraTransformCheck = PM::get().TransformationCheckerRegistrar.create(
		"CounterTransformationChecker"
	);
	icp.transformationCheckers.push_back(extraTransformCheck);
	
	addFilter("BoundTransformationChecker", {
			{"maxRotationNorm", toParam(1.0)},
			{"maxTranslationNorm", toParam(1.0)}
		}
	);
	validate2dTransformation();
}

//---------------------------
// Transformation
//---------------------------
TEST(Transformation, RigidTransformation)
{
	std::shared_ptr<PM::Transformation> rigidTrans;
	rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

	//-------------------------------------
	// Construct a 3D non-orthogonal matrix
	PM::Matrix T_3D = PM::Matrix::Identity(4,4);
	//T_3D(0,0) = 2.3;
	//T_3D(0,1) = 0.03;
	T_3D << 0.99935116,  0.13669771,  0.03436585,  1.71138524,
	       -0.02633967,  0.99326295, -0.04907545, -0.10860933,
				 -0.03615132,  0.04400287,  0.99820427, -0.04454497,
				  0.        ,  0.        ,  0.        ,  1.;

	EXPECT_FALSE(rigidTrans->checkParameters(T_3D));

	EXPECT_THROW(rigidTrans->compute(data3D, T_3D), TransformationError);

	// Check stability over iterations
	for(int i = 0; i < 10; i++)
	{
		T_3D = rigidTrans->correctParameters(T_3D);
		ASSERT_TRUE(rigidTrans->checkParameters(T_3D));
	}

	//-------------------------------------
	// Construct a 2D non-orthogonal matrix
	PM::Matrix T_2D_non_ortho = PM::Matrix::Identity(3,3);
	T_2D_non_ortho(0,0) = 0.8;
	T_2D_non_ortho(0,1) = -0.5;
	T_2D_non_ortho(1,0) = 0.5;
	T_2D_non_ortho(1,1) = 0.8;

	EXPECT_FALSE(rigidTrans->checkParameters(T_2D_non_ortho));

	EXPECT_THROW(rigidTrans->compute(data2D, T_2D_non_ortho), TransformationError);

	// Check stability over iterations
	for(int i = 0; i < 10; i++)
	{
		T_2D_non_ortho = rigidTrans->correctParameters(T_2D_non_ortho);
		EXPECT_TRUE(rigidTrans->checkParameters(T_2D_non_ortho));
	}

	//-------------------------------------
	// Construct a 2D reflection matrix
	PM::Matrix T_2D_reflection = PM::Matrix::Identity(3,3);
	T_2D_reflection(1,1) = -1;

	EXPECT_THROW(rigidTrans->correctParameters(T_2D_reflection), TransformationError);
}
